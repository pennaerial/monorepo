import asyncio
from mavsdk import System
import math

def calculate_heading(lat1, lon1, lat2, lon2):
    """
    Calculate the heading (bearing) from the current position to the new position.

    Args:
        lat1 (float): Latitude of the current position (in degrees).
        lon1 (float): Longitude of the current position (in degrees).
        lat2 (float): Latitude of the new position (in degrees).
        lon2 (float): Longitude of the new position (in degrees).

    Returns:
        float: The heading (bearing) in degrees from the current position to the new position.
    """
    # Convert latitudes and longitudes from degrees to radians
    lat1 = math.radians(lat1)
    lon1 = math.radians(lon1)
    lat2 = math.radians(lat2)
    lon2 = math.radians(lon2)

    # Compute the difference in longitude
    delta_lon = lon2 - lon1

    # Compute the bearing using the bearing formula
    x = math.sin(delta_lon) * math.cos(lat2)
    y = math.cos(lat1) * math.sin(lat2) - math.sin(lat1) * math.cos(lat2) * math.cos(delta_lon)

    # atan2 gives the result in radians, convert to degrees
    bearing = math.atan2(x, y)
    bearing = math.degrees(bearing)

    # Normalize the bearing to be within 0-360 degrees
    bearing = (bearing + 360) % 360

    return bearing



async def navigate(gps_points, drone):
    """
    Navigate the drones to the given list of GPS points based on timestamps.

    Args:
        gps_points (list of tuples): List of GPS coordinates as (latitude, longitude, altitude).
        timestamps (list of (float, int)): Corresponding timestamp (in seconds), drone-id pairs.
        drones (list of mavsdk.System): List of connected drones.
    """
    print("Starting navigation...")

    for i, point in enumerate(gps_points):

        latitude, longitude = point
        async for position in drone.telemetry.position():
            current_lat = position.latitude_deg
            current_lon = position.longitude_deg
            altitude = position.absolute_altitude_m
            break

        heading = calculate_heading(current_lat, current_lon, latitude, longitude)

        print(f"Moving to new location: Latitude: {latitude}, Longitude: {longitude}, Altitude: {altitude}")

        await drone.action.goto_location(latitude, longitude, position.absolute_altitude_m, heading)
        # Wait for a few seconds to allow the drone to move

        await asyncio.sleep(20)

    print("Finished navigation.")

async def run():
    # Connect to the Gazebo simulated PX4
    drone = System()

    # Connect the drone
    await drone.connect(system_address="udp://:14540")

    print("Waiting for drone to connect...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print("Drone connected!")
            break

    # Arm the drone
    print("Arming the drone...")
    await drone.action.arm()

    # Ensure the drone is in position control mode
    print("Setting drone to position control mode...")
    await drone.action.set_takeoff_altitude(5)
    await drone.action.takeoff()
    await asyncio.sleep(20)

    print("Take off done!")

    # Example usage of the navigate function with multiple GPS points
    gps_points = [(47.3980000, 8.5460000)]

    await navigate(gps_points, drone)

    # Land the drone after navigation
    print("Landing the drone...")
    await drone.action.land()

if __name__ == "__main__":
    loop = asyncio.get_event_loop()
    loop.run_until_complete(run())


