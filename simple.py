#!/usr/bin/env python3
import asyncio
from mavsdk import System

# These must match the PX4 MAVLink "remote port" logs:
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


async def arm_and_takeoff(name: str,
                          drone: System,
                          delay_before: float = 0.0,
                          alt_m: float = 10.0) -> None:
    if delay_before > 0.0:
        print(f"[{name}] Waiting {delay_before} s before takeoff ...")
        await asyncio.sleep(delay_before)

    print(f"[{name}] Arming ...")
    await drone.action.arm()

    print(f"[{name}] Takeoff to ~{alt_m} m ...")
    await drone.action.takeoff()


async def main():
    # Create two independent MAVSDK Systems using DIFFERENT gRPC ports
    leader = System(port=LEADER_MAVSDK_PORT)
    follower = System(port=FOLLOWER_MAVSDK_PORT)

    # Connect both in parallel
    await asyncio.gather(
        connect_and_wait("leader",   leader,   LEADER_UDP_ADDR),
        connect_and_wait("follower", follower, FOLLOWER_UDP_ADDR),
    )

    # Leader takes off immediately
    await arm_and_takeoff("leader", leader, delay_before=0.0, alt_m=10.0)

    # Follower (camera drone) takes off 5 seconds later
    await arm_and_takeoff("follower", follower, delay_before=5.0, alt_m=10.0)

    print("[main] Both should now be hovering. Waiting 60 s ...")
    await asyncio.sleep(60.0)

    print("[main] Landing both drones ...")
    await leader.action.land()
    await follower.action.land()
    await asyncio.sleep(10.0)
    print("[main] Done.")


if __name__ == "__main__":
    asyncio.run(main())
