#!/usr/bin/env python3
import asyncio
import math
import time

from mavsdk import System
from mavsdk.mission import MissionItem, MissionPlan, MissionError
from grpc.aio import AioRpcError

# --- ROS 2 / Gazebo imports ---
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix

# --- Plotting ---
import matplotlib.pyplot as plt

# === HOW TO RUN EXTERNAL PIECES ===
#
# 1) Start PX4 SITL with your two vehicles as you already do.
#
# 2) Start Gazebo→ROS bridge for leader NavSat (in another terminal):
#
#   ros2 run ros_gz_bridge parameter_bridge \
#     /world/default/model/x500_depth_2/link/base_link/sensor/navsat_sensor/navsat@sensor_msgs/msg/NavSatFix[gz.msgs.NavSat
#
# 3) THEN run this Python script.
#
# No need to run mavsdk_server manually: each System() below starts its own
# mavsdk_server just like your original script.

# === CONFIG ===

# These must match the PX4 MAVLink "remote port" values in the PX4 logs:
LEADER_UDP_ADDR   = "udp://:14542"  # leader PX4 instance (x500_depth_2, -i 2)
FOLLOWER_UDP_ADDR = "udp://:14541"  # follower PX4 instance (x500_depth, -i 1)

# Separate MAVSDK server gRPC ports for each drone
LEADER_MAVSDK_PORT   = 50051
FOLLOWER_MAVSDK_PORT = 50052

# Gazebo->ROS NavSat topic for the leader's navsat sensor
GAZEBO_NAVSAT_TOPIC = (
    "/world/default/model/x500_depth_2/link/base_link/"
    "sensor/navsat_sensor/navsat"
)

EARTH_RADIUS_M = 6378137.0


# === COORDINATE HELPERS ===

def latlon_to_ned(lat, lon, alt, home_lat, home_lon, home_alt):
    """
    Convert (lat, lon, alt) to local NED (North, East, Down) in meters,
    relative to (home_lat, home_lon, home_alt).
    """
    d_lat = math.radians(lat - home_lat)
    d_lon = math.radians(lon - home_lon)

    north = d_lat * EARTH_RADIUS_M
    east = d_lon * EARTH_RADIUS_M * math.cos(math.radians(home_lat))
    down = home_alt - alt  # positive down

    return north, east, down


def ned_to_enu(north, east, down):
    """
    PX4 uses NED, Gazebo world is ENU-like (x=East, y=North, z=Up).
    This converts NED -> ENU.
    """
    x_enu = east
    y_enu = north
    z_enu = -down
    return x_enu, y_enu, z_enu


# === ROS 2 NODE TO LISTEN TO GAZEBO NAVSAT ===

class LeaderNavSatListener(Node):
    """
    Simple ROS 2 node that subscribes to the leader's NavSat data from Gazebo
    (via ros_gz_bridge) and stores the latest message.
    """
    def __init__(self):
        super().__init__("leader_navsat_listener")
        self.navsat_msg = None

        self.create_subscription(
            NavSatFix,
            GAZEBO_NAVSAT_TOPIC,
            self._navsat_cb,
            10,
        )
        self.get_logger().info(f"Subscribed to Gazebo NavSat topic: {GAZEBO_NAVSAT_TOPIC}")

    def _navsat_cb(self, msg: NavSatFix):
        self.navsat_msg = msg


async def spin_ros(node: Node):
    """
    Periodically spin the ROS 2 node inside asyncio.
    """
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            await asyncio.sleep(0.01)
    except asyncio.CancelledError:
        pass


# === MAVSDK CONNECTION & MISSION HELPERS (original style) ===

async def connect_and_wait(name: str, drone: System, udp_addr: str) -> None:
    """
    Connect a given System() to its UDP endpoint and wait for:
      - connection
      - global position & home
    """
    print(f"[{name}] Connecting on {udp_addr} (mavsdk_server port {drone._port if hasattr(drone, '_port') else 'unknown'}) ...")
    await drone.connect(system_address=udp_addr)

    # Wait until connected
    async for state in drone.core.connection_state():
        if state.is_connected:
            print(f"[{name}] Connected")
            break

    # Wait for GPS & home
    print(f"[{name}] Waiting for global position / home ...")
    async for h in drone.telemetry.health():
        if h.is_global_position_ok and h.is_home_position_ok:
            print(f"[{name}] Global position OK")
            break


def wp(lat: float, lon: float, rel_alt: float) -> MissionItem:
    """Create a single mission waypoint."""
    return MissionItem(
        latitude_deg=lat,
        longitude_deg=lon,
        relative_altitude_m=rel_alt,
        speed_m_s=5.0,
        is_fly_through=False,
        gimbal_pitch_deg=0.0,
        gimbal_yaw_deg=0.0,
        camera_action=MissionItem.CameraAction.NONE,
        loiter_time_s=0.0,
        camera_photo_interval_s=0.0,
        acceptance_radius_m=2.0,
        yaw_deg=float("nan"),
        camera_photo_distance_m=0.0,
        vehicle_action=MissionItem.VehicleAction.NONE,
    )


def build_square_mission(home_lat: float, home_lon: float,
                         rel_alt: float = 10.0) -> MissionPlan:
    """
    Simple square mission around home (~40 m side length).
    """
    dlat = 0.00036  # ~40 m north/south near Zurich lat
    dlon = 0.00036  # ~40 m east/west

    items = [
        wp(home_lat,          home_lon,          rel_alt),
        wp(home_lat + dlat,   home_lon,          rel_alt),
        wp(home_lat + dlat,   home_lon + dlon,   rel_alt),
        wp(home_lat,          home_lon + dlon,   rel_alt),
        wp(home_lat,          home_lon,          rel_alt),
    ]
    return MissionPlan(items)


async def upload_square_mission(name: str, drone: System) -> None:
    """
    Get home lat/lon and upload a square mission around it.
    Includes a simple retry in case PX4 is briefly BUSY.
    """
    # Get home position (we already waited for home to be OK)
    async for home in drone.telemetry.home():
        home_lat = home.latitude_deg
        home_lon = home.longitude_deg
        break

    print(f"[{name}] home lat={home_lat}, lon={home_lon}")
    mission_plan = build_square_mission(home_lat, home_lon, rel_alt=10.0)

    await drone.mission.set_return_to_launch_after_mission(True)

    # Clear + upload with a small retry loop for BUSY
    for attempt in range(1, 4):
        try:
            print(f"[{name}] Clearing old mission (attempt {attempt}) ...")
            await drone.mission.clear_mission()

            print(f"[{name}] Uploading mission (attempt {attempt}) ...")
            await drone.mission.upload_mission(mission_plan)

            print(f"[{name}] Mission uploaded successfully.")
            return
        except MissionError as e:
            print(f"[{name}] MissionError on upload: {e._result.result}")
            await asyncio.sleep(1.0)

    print(f"[{name}] ERROR: Failed to upload mission after retries.")


# === PLOTTING ===

def plot_enu_time(
    t_list,
    x_px4_list, y_px4_list, z_px4_list,
    x_gz_list,  y_gz_list,  z_gz_list,
):
    if not t_list:
        print("[plot] No data collected, skipping plot.")
        return

    fig, axes = plt.subplots(3, 1, sharex=True, figsize=(10, 8))

    axes[0].plot(t_list, x_px4_list, label="PX4 X")
    axes[0].plot(t_list, x_gz_list,  label="Gazebo X", linestyle="--")
    axes[0].set_ylabel("X ENU [m]")
    axes[0].grid(True)
    axes[0].legend()

    axes[1].plot(t_list, y_px4_list, label="PX4 Y")
    axes[1].plot(t_list, y_gz_list,  label="Gazebo Y", linestyle="--")
    axes[1].set_ylabel("Y ENU [m]")
    axes[1].grid(True)
    axes[1].legend()

    axes[2].plot(t_list, z_px4_list, label="PX4 Z")
    axes[2].plot(t_list, z_gz_list,  label="Gazebo Z", linestyle="--")
    axes[2].set_ylabel("Z ENU [m]")
    axes[2].set_xlabel("Time [s]")
    axes[2].grid(True)
    axes[2].legend()

    fig.suptitle("Leader ENU position: PX4 vs Gazebo")
    plt.tight_layout()
    plt.show()


# === COMPARISON TASK USING THE SAME LEADER System() ===

async def compare_leader_px4_vs_gazebo(
    leader: System,
    node: LeaderNavSatListener,
    home_lat: float,
    home_lon: float,
    home_alt: float,
) -> None:
    """
    Background task: collect ENU trajectories for PX4 and Gazebo
    and, when cancelled, plot them.
    """
    print("\n=== Collecting position data for ENU plot (leader PX4 vs Gazebo) ===\n")

    t0 = time.monotonic()
    t_list = []
    x_px4_list, y_px4_list, z_px4_list = [], [], []
    x_gz_list,  y_gz_list,  z_gz_list  = [], [], []

    try:
        while True:
            pos = await leader.telemetry.position().__anext__()
            lat_px4 = pos.latitude_deg
            lon_px4 = pos.longitude_deg
            alt_px4 = pos.absolute_altitude_m

            n_px4, e_px4, d_px4 = latlon_to_ned(
                lat_px4, lon_px4, alt_px4, home_lat, home_lon, home_alt
            )
            x_px4, y_px4, z_px4 = ned_to_enu(n_px4, e_px4, d_px4)

            navsat_msg = node.navsat_msg
            if navsat_msg is not None:
                lat_gz = navsat_msg.latitude
                lon_gz = navsat_msg.longitude
                alt_gz = navsat_msg.altitude

                n_gz, e_gz, d_gz = latlon_to_ned(
                    lat_gz, lon_gz, alt_gz, home_lat, home_lon, home_alt
                )
                x_gz, y_gz, z_gz = ned_to_enu(n_gz, e_gz, d_gz)

                t = time.monotonic() - t0
                t_list.append(t)
                x_px4_list.append(x_px4)
                y_px4_list.append(y_px4)
                z_px4_list.append(z_px4)
                x_gz_list.append(x_gz)
                y_gz_list.append(y_gz)
                z_gz_list.append(z_gz)

            await asyncio.sleep(0.1)

    except asyncio.CancelledError:
        print("[compare] Task cancelled, creating ENU position plots ...")
        plot_enu_time(
            t_list,
            x_px4_list, y_px4_list, z_px4_list,
            x_gz_list,  y_gz_list,  z_gz_list,
        )
        raise


# === MAIN ===

async def main():
    # ROS 2 node & spinning task
    rclpy.init()
    navsat_node = LeaderNavSatListener()
    ros_task = asyncio.create_task(spin_ros(navsat_node))

    # Create two independent MAVSDK Systems using DIFFERENT gRPC ports
    # (same as your original working script)
    leader = System(port=LEADER_MAVSDK_PORT)
    follower = System(port=FOLLOWER_MAVSDK_PORT)

    # Connect both in parallel (same pattern as original)
    await asyncio.gather(
        connect_and_wait("leader",   leader,   LEADER_UDP_ADDR),
        connect_and_wait("follower", follower, FOLLOWER_UDP_ADDR),
    )

    # Get leader home (for ENU frame used in comparison)
    async for home in leader.telemetry.home():
        home_lat = home.latitude_deg
        home_lon = home.longitude_deg
        home_alt = home.absolute_altitude_m
        print(
            f"[leader HOME] lat={home_lat:.7f}, lon={home_lon:.7f}, "
            f"alt={home_alt:.2f} m"
        )
        break

    # Start comparison task (collects data for plotting)
    compare_task = asyncio.create_task(
        compare_leader_px4_vs_gazebo(
            leader, navsat_node, home_lat, home_lon, home_alt
        )
    )

    # Upload the same square mission to both
    await asyncio.gather(
        upload_square_mission("leader",   leader),
        upload_square_mission("follower", follower),
    )

    # Start leader mission
    try:
        print("[leader] Arming ...")
        await leader.action.arm()
        print("[leader] Starting mission ...")
        await leader.mission.start_mission()
    except AioRpcError as e:
        print(f"[leader] gRPC error while arming/starting mission: {e}")
        return

    # Wait 3 seconds, then start follower mission
    await asyncio.sleep(3.0)

    try:
        print("[follower] Arming ...")
        await follower.action.arm()
        print("[follower] Starting mission ...")
        await follower.mission.start_mission()
    except AioRpcError as e:
        print(f"[follower] gRPC error while arming/starting mission: {e}")
        return

    # Let them fly for a while (e.g. 2 minutes)
    print("[main] Letting missions run for 2 minutes ...")
    await asyncio.sleep(120.0)

    # Request RTL for both (they may already RTL after mission, but this is safe)
    print("[main] Requesting RTL for both drones ...")
    await leader.action.return_to_launch()
    await follower.action.return_to_launch()
    await asyncio.sleep(20.0)
    print("[main] Done with missions.")

    # Cleanup + trigger plotting via task cancel
    compare_task.cancel()
    ros_task.cancel()
    try:
        await compare_task
    except asyncio.CancelledError:
        pass
    try:
        await ros_task
    except asyncio.CancelledError:
        pass

    rclpy.shutdown()
    print("[main] ROS 2 shutdown complete.")


if __name__ == "__main__":
    asyncio.run(main())
