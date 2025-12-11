#!/usr/bin/env python3
import asyncio
from mavsdk import System
from mavsdk.mission import MissionItem, MissionPlan, MissionError

# These must match the PX4 MAVLink "remote port" values in the PX4 logs:
LEADER_UDP_ADDR   = "udp://:14542"  # leader PX4 instance (e.g. x500, -i 2)
FOLLOWER_UDP_ADDR = "udp://:14541"  # follower PX4 instance (x500_depth, -i 1)

# Separate MAVSDK server gRPC ports for each drone
LEADER_MAVSDK_PORT   = 50051
FOLLOWER_MAVSDK_PORT = 50052


async def connect_and_wait(name: str, drone: System, udp_addr: str) -> None:
    """
    Connect a given System() to its UDP endpoint and wait for:
      - connection
      - global position & home
    """
    print(f"[{name}] Connecting on {udp_addr} ...")
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


async def main():
    # Create two independent MAVSDK Systems using DIFFERENT gRPC ports
    leader = System(port=LEADER_MAVSDK_PORT)
    follower = System(port=FOLLOWER_MAVSDK_PORT)

    # Connect both in parallel
    await asyncio.gather(
        connect_and_wait("leader",   leader,   LEADER_UDP_ADDR),
        connect_and_wait("follower", follower, FOLLOWER_UDP_ADDR),
    )

    # Upload the same square mission to both
    await asyncio.gather(
        upload_square_mission("leader",   leader),
        upload_square_mission("follower", follower),
    )

    # Start leader mission
    print("[leader] Arming ...")
    await leader.action.arm()
    print("[leader] Starting mission ...")
    await leader.mission.start_mission()

    # Wait 5 seconds, then start follower mission
    print("[follower] Waiting 5 s before starting mission ...")
    await asyncio.sleep(5.0)

    print("[follower] Arming ...")
    await follower.action.arm()
    print("[follower] Starting mission ...")
    await follower.mission.start_mission()

    # Let them fly for a while (e.g. 2 minutes)
    print("[main] Letting missions run for 2 minutes ...")
    await asyncio.sleep(120.0)

    # Request RTL for both (they may already RTL after mission, but this is safe)
    print("[main] Requesting RTL for both drones ...")
    await leader.action.return_to_launch()
    await follower.action.return_to_launch()
    await asyncio.sleep(20.0)
    print("[main] Done.")


if __name__ == "__main__":
    asyncio.run(main())
