#!/usr/bin/env python3
import asyncio
import math
import time
import csv
from dataclasses import dataclass
from typing import Optional, Tuple

from mavsdk import System
from mavsdk.mission import MissionItem, MissionPlan, MissionError
from grpc.aio import AioRpcError

# ROS 2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

# OpenCV / vision
import cv2
import numpy as np
from cv_bridge import CvBridge

# === CONFIG ===

# These must match the PX4 MAVLink "remote port" values in the PX4 logs:
LEADER_UDP_ADDR   = "udp://:14542"  # leader PX4 instance (e.g. x500, -i 2)
FOLLOWER_UDP_ADDR = "udp://:14541"  # follower PX4 instance (e.g. x500_depth, -i 1)

# Separate MAVSDK server gRPC ports for each drone
LEADER_MAVSDK_PORT   = 50051
FOLLOWER_MAVSDK_PORT = 50052

# ROS topic where the follower camera image is published (after ros_gz_bridge)
FOLLOWER_CAMERA_TOPIC = "/world/default/model/x500_depth_1/link/camera_link/sensor/IMX214/image"

# Camera extrinsics: camera frame C relative to follower body frame B
# For now we assume camera is at follower origin (no offset) and aligned with body.
CAM_R_B = [
    [1.0, 0.0, 0.0],
    [0.0, 1.0, 0.0],
    [0.0, 0.0, 1.0],
]
CAM_T_B = [0.0, 0.0, 0.0]

EARTH_RADIUS_M = 6378137.0

# Camera intrinsics from your CameraInfo (k:)
CAMERA_MATRIX = np.array(
    [[1397.2235870361328,       0.0, 960.0],
     [      0.0,          1397.2235298156738, 540.0],
     [      0.0,                 0.0,         1.0]],
    dtype=np.float32,
)

# cv_bridge instance
_BRIDGE = CvBridge()

# How far along the ray to place the leader (meters)
RAY_RANGE_M = 10.0  # adjust as needed

# Minimum blob area (pixels) to be considered a “drone”
MIN_BLOB_AREA = 200.0

# === LOGGING FOR PLOTS ===
vision_log = {
    "t":       [],
    "est_x":   [],
    "est_y":   [],
    "est_z":   [],
    "true_x":  [],
    "true_y":  [],
    "true_z":  [],
}

# === BASIC LINEAR ALGEBRA HELPERS ===

def mat_vec_mul(R, v):
    """Multiply 3x3 matrix R by 3x1 vector v."""
    return [
        R[0][0]*v[0] + R[0][1]*v[1] + R[0][2]*v[2],
        R[1][0]*v[0] + R[1][1]*v[1] + R[1][2]*v[2],
        R[2][0]*v[0] + R[2][1]*v[1] + R[2][2]*v[2],
    ]


def vec_add(a, b):
    return [a[0]+b[0], a[1]+b[1], a[2]+b[2]]


# === COORDINATE HELPERS ===

def latlon_to_ned(lat, lon, alt, home_lat, home_lon, home_alt):
    """
    Convert (lat, lon, alt) to local NED (North, East, Down) in meters,
    relative to (home_lat, home_lon, home_alt).
    """
    d_lat = math.radians(lat - home_lat)
    d_lon = math.radians(lon - home_lon)

    north = d_lat * EARTH_RADIUS_M
    east  = d_lon * EARTH_RADIUS_M * math.cos(math.radians(home_lat))
    down  = home_alt - alt  # positive down

    return north, east, down


def ned_to_enu(north, east, down):
    """
    PX4 uses NED, Gazebo world is ENU-like (x=East, y=North, z=Up).
    This converts NED -> ENU.
    """
    x_enu = east
    y_enu = north
    z_enu = -down
    return [x_enu, y_enu, z_enu]


def quaternion_to_rot_wb(w: float, x: float, y: float, z: float):
    """
    Convert quaternion (w,x,y,z) to rotation matrix R_WB (body->world/ENU).
    This assumes PX4's attitude_quaternion is in NED/ENU consistent frame.
    """
    ww = w*w
    xx = x*x
    yy = y*y
    zz = z*z

    wx = w*x
    wy = w*y
    wz = w*z
    xy = x*y
    xz = x*z
    yz = y*z

    R = [[0.0]*3 for _ in range(3)]
    R[0][0] = ww + xx - yy - zz
    R[0][1] = 2*(xy - wz)
    R[0][2] = 2*(xz + wy)

    R[1][0] = 2*(xy + wz)
    R[1][1] = ww - xx + yy - zz
    R[1][2] = 2*(yz - wx)

    R[2][0] = 2*(xz - wy)
    R[2][1] = 2*(yz + wx)
    R[2][2] = ww - xx - yy + zz

    return R


# === ROS 2 CAMERA LISTENER ===

class FollowerCameraListener(Node):
    """
    ROS 2 node that subscribes to the follower's camera image.
    We just store the latest Image in memory.
    """
    def __init__(self):
        super().__init__("follower_camera_listener")
        self.image_msg: Optional[Image] = None

        self.create_subscription(
            Image,
            FOLLOWER_CAMERA_TOPIC,
            self._image_cb,
            10,
        )
        self.get_logger().info(f"Subscribed to follower camera topic: {FOLLOWER_CAMERA_TOPIC}")

    def _image_cb(self, msg: Image):
        self.image_msg = msg


async def spin_ros(nodes):
    """
    Periodically spin multiple ROS 2 nodes inside asyncio.
    """
    try:
        while rclpy.ok():
            for n in nodes:
                rclpy.spin_once(n, timeout_sec=0.0)
            await asyncio.sleep(0.01)
    except asyncio.CancelledError:
        pass


# === MAVSDK HELPERS ===

async def connect_and_wait(name: str, drone: System, udp_addr: str) -> None:
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
    dlat = 0.00036
    dlon = 0.00036

    items = [
        wp(home_lat,          home_lon,          rel_alt),
        wp(home_lat + dlat,   home_lon,          rel_alt),
        wp(home_lat + dlat,   home_lon + dlon,   rel_alt),
        wp(home_lat,          home_lon + dlon,   rel_alt),
        wp(home_lat,          home_lon,          rel_alt),
    ]
    return MissionPlan(items)


async def upload_square_mission(name: str, drone: System) -> None:
    async for home in drone.telemetry.home():
        home_lat = home.latitude_deg
        home_lon = home.longitude_deg
        break

    print(f"[{name}] home lat={home_lat}, lon={home_lon}")
    mission_plan = build_square_mission(home_lat, home_lon, rel_alt=10.0)

    await drone.mission.set_return_to_launch_after_mission(True)

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


# === SHARED FOLLOWER + LEADER STATE ===

@dataclass
class FollowerState:
    lat: Optional[float] = None
    lon: Optional[float] = None
    alt: Optional[float] = None
    qw:  Optional[float] = None
    qx:  Optional[float] = None
    qy:  Optional[float] = None
    qz:  Optional[float] = None


@dataclass
class LeaderState:
    lat: Optional[float] = None
    lon: Optional[float] = None
    alt: Optional[float] = None


async def follower_position_task(follower: System, state: FollowerState):
    async for pos in follower.telemetry.position():
        state.lat = pos.latitude_deg
        state.lon = pos.longitude_deg
        state.alt = pos.absolute_altitude_m
        await asyncio.sleep(0.01)


async def follower_attitude_task(follower: System, state: FollowerState):
    async for att in follower.telemetry.attitude_quaternion():
        state.qw = att.w
        state.qx = att.x
        state.qy = att.y
        state.qz = att.z
        await asyncio.sleep(0.01)


async def leader_position_task(leader: System, state: LeaderState):
    async for pos in leader.telemetry.position():
        state.lat = pos.latitude_deg
        state.lon = pos.longitude_deg
        state.alt = pos.absolute_altitude_m
        await asyncio.sleep(0.01)


# === SIMPLE VISION: GENERIC BLOB DETECTION ===

def estimate_leader_direction_from_image(image_msg: Image) -> Optional[np.ndarray]:
    """
    Simple vision technique:
      - Convert image to gray
      - Blur + Otsu threshold
      - Find contours
      - Ignore tiny blobs and blobs touching the border
      - Take the largest remaining blob's centroid as the leader pixel
      - Convert pixel (u,v) into a unit direction vector in camera frame.

    Returns:
      dir_C: np.array([x_c, y_c, z_c]) unit vector in camera frame,
             or None if no blob detected.
    """
    try:
        cv_image = _BRIDGE.imgmsg_to_cv2(image_msg, desired_encoding="bgr8")
    except Exception as e:
        print(f"[vision] cv_bridge conversion failed: {e}")
        return None

    h, w = cv_image.shape[:2]
    print(f"[vision] Got image {w}x{h}")

    # Convert to grayscale and blur
    gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
    blur = cv2.GaussianBlur(gray, (5, 5), 0)

    # Otsu threshold (auto threshold)
    _, mask = cv2.threshold(
        blur, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU
    )

    # Sometimes the foreground/background are inverted. Try both and choose
    # the one with smaller total white area as "object".
    white_ratio = np.mean(mask > 0)
    if white_ratio > 0.5:
        mask = cv2.bitwise_not(mask)
        print("[vision] Inverted mask (background seemed white).")

    # Find contours
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    print(f"[vision] Found {len(contours)} contours")

    if not contours:
        print("[vision] No contours found.")
        return None

    candidates = []
    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area < MIN_BLOB_AREA:
            continue

        x, y, ww, hh = cv2.boundingRect(cnt)

        # Discard blobs touching the image border (likely background)
        if x <= 0 or y <= 0 or x+ww >= w-1 or y+hh >= h-1:
            continue

        candidates.append((area, cnt))

    if not candidates:
        print("[vision] No suitable blob candidates (either too small or on border).")
        return None

    # Choose the largest candidate
    candidates.sort(key=lambda c: c[0], reverse=True)
    area, best = candidates[0]
    print(f"[vision] Using blob with area={area:.1f}")

    M = cv2.moments(best)
    if M["m00"] == 0:
        print("[vision] Moment m00 is zero; skipping.")
        return None

    u = M["m10"] / M["m00"]
    v = M["m01"] / M["m00"]
    print(f"[vision] Blob centroid pixel: u={u:.1f}, v={v:.1f}")

    # Convert pixel (u,v) to normalized camera direction
    fx = CAMERA_MATRIX[0, 0]
    fy = CAMERA_MATRIX[1, 1]
    cx = CAMERA_MATRIX[0, 2]
    cy = CAMERA_MATRIX[1, 2]

    x = (u - cx) / fx
    y = (v - cy) / fy
    z = 1.0

    dir_c = np.array([x, y, z], dtype=np.float64)
    norm = np.linalg.norm(dir_c)
    if norm < 1e-6:
        print("[vision] Direction norm too small.")
        return None

    dir_c /= norm
    print(f"[vision] Camera direction: [{dir_c[0]:.3f}, {dir_c[1]:.3f}, {dir_c[2]:.3f}]")

    return dir_c


# === CAMERA-BASED LEADER POSITION (BEARING + FIXED RANGE) ===

async def camera_based_leader_pose_task(
    follower_state: FollowerState,
    leader_state: LeaderState,
    camera_node: FollowerCameraListener,
    home_lat: float,
    home_lon: float,
    home_alt: float,
    log_dict: dict,
    t0: float,
):
    """
    Periodically:
      - read latest follower PX4 position & attitude
      - read latest camera image
      - run simple blob vision to get leader direction in camera frame
      - rotate into world ENU
      - place leader at a fixed range along that ray from the follower
      - log both estimated and true leader ENU positions
    """
    print("\n=== Camera-based leader direction (blob + fixed range) ===\n")

    try:
        while True:
            # Need follower pose and attitude
            if (
                follower_state.lat is None or follower_state.lon is None or
                follower_state.alt is None or follower_state.qw is None
            ):
                print("[vision] Waiting for follower PX4 state (pos/att)...")
                await asyncio.sleep(0.2)
                continue

            # Need leader truth for logging
            if (
                leader_state.lat is None or
                leader_state.lon is None or
                leader_state.alt is None
            ):
                print("[vision] Waiting for leader PX4 state...")
                await asyncio.sleep(0.2)
                continue

            image_msg = camera_node.image_msg
            if image_msg is None:
                print("[vision] Waiting for follower camera images ...")
                await asyncio.sleep(0.2)
                continue

            dir_C = estimate_leader_direction_from_image(image_msg)
            if dir_C is None:
                print("[vision] No valid blob / direction this frame.")
                await asyncio.sleep(0.2)
                continue

            # --- Follower position in ENU ---
            n_f, e_f, d_f = latlon_to_ned(
                follower_state.lat,
                follower_state.lon,
                follower_state.alt,
                home_lat,
                home_lon,
                home_alt,
            )
            p_F_W = ned_to_enu(n_f, e_f, d_f)  # [x_F, y_F, z_F]

            # --- Follower attitude: body -> world rotation ---
            R_WB = quaternion_to_rot_wb(
                follower_state.qw,
                follower_state.qx,
                follower_state.qy,
                follower_state.qz,
            )

            # Camera direction -> body frame
            dir_C_list = dir_C.tolist()
            dir_B = mat_vec_mul(CAM_R_B, dir_C_list)

            # Body -> world
            dir_W = mat_vec_mul(R_WB, dir_B)

            # Estimated leader in world: p_L^W_est = p_F^W + R * (RAY_RANGE_M * dir_W)
            p_L_W_est = vec_add(
                p_F_W,
                [RAY_RANGE_M * dir_W[0],
                 RAY_RANGE_M * dir_W[1],
                 RAY_RANGE_M * dir_W[2]],
            )

            # --- True leader ENU from PX4 ---
            n_L, e_L, d_L = latlon_to_ned(
                leader_state.lat,
                leader_state.lon,
                leader_state.alt,
                home_lat,
                home_lon,
                home_alt,
            )
            p_L_W_true = ned_to_enu(n_L, e_L, d_L)

            # Log
            t = time.time() - t0
            log_dict["t"].append(t)
            log_dict["est_x"].append(p_L_W_est[0])
            log_dict["est_y"].append(p_L_W_est[1])
            log_dict["est_z"].append(p_L_W_est[2])
            log_dict["true_x"].append(p_L_W_true[0])
            log_dict["true_y"].append(p_L_W_true[1])
            log_dict["true_z"].append(p_L_W_true[2])

            err_x = p_L_W_true[0] - p_L_W_est[0]
            err_y = p_L_W_true[1] - p_L_W_est[1]
            err_z = p_L_W_true[2] - p_L_W_est[2]
            err_norm = math.sqrt(err_x**2 + err_y**2 + err_z**2)

            print(
                "[vision] Leader ENU | "
                f"est=({p_L_W_est[0]:6.2f},{p_L_W_est[1]:6.2f},{p_L_W_est[2]:6.2f}) m, "
                f"true=({p_L_W_true[0]:6.2f},{p_L_W_true[1]:6.2f},{p_L_W_true[2]:6.2f}) m, "
                f"|err|={err_norm:.3f} m"
            )

            await asyncio.sleep(0.2)

    except asyncio.CancelledError:
        print("[vision] Camera-based leader pose task cancelled.")
        raise


# === SAVE + PLOT RESULTS ===

def save_and_plot(log_dict: dict):
    if not log_dict["t"]:
        print("[plot] No vision data collected; skipping CSV/plot.")
        return

    # --- Save CSV ---
    filename = "leader_vision_log.csv"
    try:
        with open(filename, "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(
                ["t",
                 "est_x", "est_y", "est_z",
                 "true_x", "true_y", "true_z"]
            )
            for i in range(len(log_dict["t"])):
                writer.writerow([
                    log_dict["t"][i],
                    log_dict["est_x"][i],
                    log_dict["est_y"][i],
                    log_dict["est_z"][i],
                    log_dict["true_x"][i],
                    log_dict["true_y"][i],
                    log_dict["true_z"][i],
                ])
        print(f"[plot] Saved log to {filename}")
    except Exception as e:
        print(f"[plot] Failed to save CSV: {e}")

    # --- Try to plot with matplotlib ---
    try:
        import matplotlib.pyplot as plt
    except Exception as e:
        print("[plot] Could not import matplotlib for plotting:", e)
        print("[plot] Skipping plot. You can still use the CSV for offline plotting.")
        return

    t = np.array(log_dict["t"])
    est_x = np.array(log_dict["est_x"])
    est_y = np.array(log_dict["est_y"])
    est_z = np.array(log_dict["est_z"])
    true_x = np.array(log_dict["true_x"])
    true_y = np.array(log_dict["true_y"])
    true_z = np.array(log_dict["true_z"])

    plt.figure(figsize=(10, 8))

    plt.subplot(3, 1, 1)
    plt.plot(t, true_x, label="true_x")
    plt.plot(t, est_x, "--", label="est_x")
    plt.ylabel("X [m]")
    plt.legend()
    plt.grid(True)

    plt.subplot(3, 1, 2)
    plt.plot(t, true_y, label="true_y")
    plt.plot(t, est_y, "--", label="est_y")
    plt.ylabel("Y [m]")
    plt.legend()
    plt.grid(True)

    plt.subplot(3, 1, 3)
    plt.plot(t, true_z, label="true_z")
    plt.plot(t, est_z-10, "--", label="est_z")
    plt.xlabel("Time [s]")
    plt.ylabel("Z [m]")
    plt.legend()
    plt.grid(True)

    plt.tight_layout()
    try:
        plt.show()
    except Exception as e:
        print(f"[plot] Failed to display plot (maybe no DISPLAY): {e}")


# === MAIN ===

async def main():
    # ROS 2 init + camera node
    rclpy.init()
    cam_node = FollowerCameraListener()
    ros_task = asyncio.create_task(spin_ros([cam_node]))

    # Two MAVSDK Systems
    leader = System(port=LEADER_MAVSDK_PORT)
    follower = System(port=FOLLOWER_MAVSDK_PORT)

    await asyncio.gather(
        connect_and_wait("leader",   leader,   LEADER_UDP_ADDR),
        connect_and_wait("follower", follower, FOLLOWER_UDP_ADDR),
    )

    # Use leader home as reference frame
    async for home in leader.telemetry.home():
        home_lat = home.latitude_deg
        home_lon = home.longitude_deg
        home_alt = home.absolute_altitude_m
        print(
            f"[home] lat={home_lat:.7f}, lon={home_lon:.7f}, "
            f"alt={home_alt:.2f} m"
        )
        break

    # Follower & leader state tracking tasks
    follower_state = FollowerState()
    leader_state   = LeaderState()

    follower_pos_task = asyncio.create_task(follower_position_task(follower, follower_state))
    follower_att_task = asyncio.create_task(follower_attitude_task(follower, follower_state))
    leader_pos_task   = asyncio.create_task(leader_position_task(leader, leader_state))

    t0 = time.time()

    # Start camera-based leader pose estimation task
    vision_task = asyncio.create_task(
        camera_based_leader_pose_task(
            follower_state,
            leader_state,
            cam_node,
            home_lat,
            home_lon,
            home_alt,
            vision_log,
            t0,
        )
    )

    # Upload missions
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

    # Start follower mission after a delay
    await asyncio.sleep(3.0)
    try:
        print("[follower] Arming ...")
        await follower.action.arm()
        print("[follower] Starting mission ...")
        await follower.mission.start_mission()
    except AioRpcError as e:
        print(f"[follower] gRPC error while arming/starting mission: {e}")
        return

    # Let them fly (e.g. 2 minutes)
    print("[main] Letting missions run for 2 minutes ...")
    await asyncio.sleep(120.0)

    # RTL both
    print("[main] Requesting RTL for both drones ...")
    await leader.action.return_to_launch()
    await follower.action.return_to_launch()
    await asyncio.sleep(20.0)
    print("[main] Done with missions.")

    # Cleanup async tasks
    for task in [vision_task, follower_pos_task, follower_att_task, leader_pos_task, ros_task]:
        task.cancel()

    for task in [vision_task, follower_pos_task, follower_att_task, leader_pos_task, ros_task]:
        try:
            await task
        except asyncio.CancelledError:
            pass

    # Shutdown ROS
    rclpy.shutdown()
    print("[main] ROS 2 shutdown complete.")

    # Save CSV + (optionally) plot
    save_and_plot(vision_log)


if __name__ == "__main__":
    asyncio.run(main())
