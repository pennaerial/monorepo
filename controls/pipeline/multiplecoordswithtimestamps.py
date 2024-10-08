import asyncio
from mavsdk import System
import math

async def calculate_yaw(current_lat, current_lon, dest_lat, dest_lon):
    """Calculate the yaw angle to face the destination."""
    delta_lon = dest_lon - current_lon
    y = math.sin(delta_lon) * math.cos(dest_lat)
    x = math.cos(current_lat) * math.sin(dest_lat) - math.sin(current_lat) * math.cos(dest_lat) * math.cos(delta_lon)
    yaw = math.atan2(y, x)
    return math.degrees(yaw)

async def navigate(gps_points, timestamps, drones):
    """
    Navigate the drones to the given list of GPS points based on timestamps.

    Args:
        gps_points (list of tuples): List of GPS coordinates as (latitude, longitude, altitude).
        timestamps (list of (float, int)): Corresponding timestamp (in seconds), drone-id pairs.
        drones (list of mavsdk.System): List of connected drones.
    """
    print("Starting navigation...")
    start_time = asyncio.get_event_loop().time()

    for i, point in enumerate(gps_points):
        latitude, longitude, altitude = point
        time, drone_id = timestamps[i]
        print(f"Navigating drone {drone_id} to point {i}: {latitude}, {longitude}, {altitude} at time {time}")

        # Wait until it's time to move to this point
        await asyncio.sleep(max(0, start_time + time - asyncio.get_event_loop().time()))

        drone = drones[drone_id]

        # Get current position of the drone
        async for position in drone.telemetry.position():
            current_lat = position.latitude_deg
            current_lon = position.longitude_deg
            break

        # Calculate yaw angle
        yaw = await calculate_yaw(current_lat, current_lon, latitude, longitude)

        # Send navigation command to the drone
        await drone.action.goto_location(latitude, longitude, altitude, yaw)
        print(f"Navigation command sent to drone {drone_id}")

    print("Finished navigation.")

async def run():
    # Connect to the drone
    drone = System()
    await drone.connect(system_address="udp://:14540")
    print("Waiting for drone to connect...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print("Drone connected!")
            break

    # Arm the drone
    print("Arming drone...")
    await drone.action.arm()

    # Take off
    print("Taking off...")
    await drone.action.takeoff()

    # Wait for the drone to reach a safe altitude
    await asyncio.sleep(5)

    # Example GPS points with timestamps
    gps_points = [
        (47.3977415, 8.5455939, 10),
        (47.3982415, 8.5465939, 10),
        (47.3987415, 8.5475939, 10)
    ]
    timestamps = [(0, 0), (10, 0), (20, 0)]  # (time in seconds, drone_id)
    drones = [drone]

    # Navigate through the points
    await navigate(gps_points, timestamps, drones)

    # Wait for the drone to finish its path
    await asyncio.sleep(10)

    # Land the drone after navigation
    print("Landing...")
    await drone.action.land()

if __name__ == "__main__":
    loop = asyncio.get_event_loop()
    loop.run_until_complete(run())