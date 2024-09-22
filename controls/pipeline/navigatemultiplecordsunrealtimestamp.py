import time
import asyncio
from mavsdk import System
import math
from mavsdk.offboard import OffboardError, VelocityNedYaw

async def navigate_with_dynamic_velocity(drone, target_lat, target_lon, target_alt, heading):
    """
    Continuously adjust the drone's velocity while navigating to a target GPS point.

    Args:
        drone (mavsdk.System): The drone instance.
        target_lat (float): Target latitude in degrees.
        target_lon (float): Target longitude in degrees.
        target_alt (float): Target altitude in meters.
    """
    max_velocity = 10.0  # Maximum velocity in m/s
    distance_threshold = 0.5 # Stop the drone when within 1 meter of the target

    await drone.offboard.set_velocity_ned(VelocityNedYaw(
        0.0,  # Initial north velocity
        0.0,   # Initial east velocity
        0.0,   # Initial altitude (downward) velocity
        heading     # Initial yaw (heading)
    ))
    await drone.offboard.start()

    await asyncio.sleep(1)

    while True:
        # Get the current position
        async for position in drone.telemetry.position():
            current_lat = position.latitude_deg
            current_lon = position.longitude_deg
            current_alt = position.absolute_altitude_m
            break

        # Calculate the distance to the target using the Haversine formula (lat/lon) and altitude difference
        distance = calculate_distance(current_lat, current_lon, current_alt, target_lat, target_lon, target_alt)
        print(f"Current distance to target: {distance} meters")

        # If we're close enough, stop the drone
        if distance < distance_threshold:
            print("Target reached, stopping the drone.")
            await drone.offboard.stop()
            break

        # Calculate the proportional velocity (slower as we get closer)
        proportional_velocity = max(min(distance / 4, max_velocity), 0.4)  # Cap min velocity to avoid getting stuck

        # Get heading towards the target
        heading = calculate_heading(current_lat, current_lon, target_lat, target_lon)

        print(target_lat, target_lon, target_alt)

        print(f"Setting velocity towards target: {proportional_velocity} m/s, heading: {heading}")

        # Set the velocity (in body or NED frame as desired, here using NED)
        await drone.offboard.set_velocity_ned(VelocityNedYaw(
            proportional_velocity * math.cos(math.radians(heading)),  # north velocity
            proportional_velocity * math.sin(math.radians(heading)),  # east velocity
            0.0,  # Adjust as necessary for altitude control (down velocity)
            heading  # Keep yaw aligned
        ))

        # Wait for a short time before recalculating (control loop timing)
        await asyncio.sleep(0.5)

def calculate_distance(lat1, lon1, alt1, lat2, lon2, alt2):
    """
    Calculate the 3D distance between two GPS points (lat, lon, alt).

    Args:
        lat1 (float): Latitude of point 1 in degrees.
        lon1 (float): Longitude of point 1 in degrees.
        alt1 (float): Altitude of point 1 in meters.
        lat2 (float): Latitude of point 2 in degrees.
        lon2 (float): Longitude of point 2 in degrees.
        alt2 (float): Altitude of point 2 in meters.

    Returns:
        float: Distance between the two points in meters.
    """
    R = 6371e3  # Earth's radius in meters
    phi1 = math.radians(lat1)
    phi2 = math.radians(lat2)
    delta_phi = math.radians(lat2 - lat1)
    delta_lambda = math.radians(lon2 - lon1)

    # Haversine formula for distance between lat/lon points
    a = math.sin(delta_phi / 2) ** 2 + math.cos(phi1) * math.cos(phi2) * math.sin(delta_lambda / 2) ** 2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
    horizontal_distance = R * c

    # Use Pythagoras to combine horizontal distance and altitude difference
    alt_difference = alt2 - alt1

    alt_difference = 0 #TODO: ACCOUNT FOR UP AND DOWN VELOCITY LATER

    distance = math.sqrt(horizontal_distance ** 2 + alt_difference ** 2)

    return distance


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


async def navigate(gps_points, timestamps, drone):
    """
    Navigate the drones to the given list of GPS points based on timestamps.
    Args:
        gps_points (list of tuples): List of GPS coordinates as (latitude, longitude, altitude).
        timestamps (list of (float, int)): Corresponding timestamp (in seconds), drone-id pairs.
        drones (list of mavsdk.System): List of connected drones.
    """

    start_time = time.time()  # Start the timer
    print(f"Start time: {start_time}")

    print("Starting navigation...")
    for i, point in enumerate(gps_points):
        latitude, longitude = point
        time, drone_id = timestamps[i]

        async for position in drone.telemetry.position():
            current_lat = position.latitude_deg
            current_lon = position.longitude_deg
            altitude = position.absolute_altitude_m
            break

        heading = calculate_heading(current_lat, current_lon, latitude, longitude)
        print(f"Moving to new location: Latitude: {latitude}, Longitude: {longitude}, Altitude: {altitude}")

        await navigate_with_dynamic_velocity(drone, latitude, longitude, altitude, heading)
        # Wait for a few seconds to allow the drone to move
        # Wait for the specified time before sending the next command

        #TODO: GET THE TIMESTAMPS ACCURATE TO BEFORE THE DRONE MOVES VS AFTER THE DRONE GETS THERE
        if i + 1 < len(timestamps):
            next_time = timestamps[i + 1][0]
            wait_time = next_time - time
            print(f"Waiting {wait_time} seconds for the next move...")
            await asyncio.sleep(wait_time)
        else:
            await asyncio.sleep(15)


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
    gps_points = [(47.3977415, 8.5455939), (47.3980000, 8.5460000), (47.3977415, 8.5455939)]
    timestamps = [(0, 1), (5, 1), (20, 1)]

    await navigate(gps_points, timestamps, drone)

    # Land the drone after navigation
    print("Landing the drone...")
    await drone.action.land()

if __name__ == "__main__":
    loop = asyncio.get_event_loop()
    loop.run_until_complete(run())


