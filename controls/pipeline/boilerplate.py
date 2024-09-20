import asyncio
from mavsdk import System

async def connect_drone(system_address):
    """
    Connects to a drone at the given system address.
    
    Args:
        system_address (str): The MAVLink system address for the drone.
    
    Returns:
        System: Connected drone instance.
    """
    drone = System()
    await drone.connect(system_address=system_address)
    
    print(f"Connecting to drone at {system_address}...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print(f"Drone connected at {system_address}!")
            break
    return drone

async def arm_and_takeoff(drone):
    """
    Arms the drone and takes off to a default altitude.
    
    Args:
        drone (mavsdk.System): The connected drone.
    """
    print("-- Arming")
    await drone.action.arm()

    print("-- Taking off")
    await drone.action.takeoff()
    await asyncio.sleep(5)

    return drone

async def navigate(gps_point, drone):
    """
    Navigate the drone to the given GPS point.
    
    Args:
        gps_point (tuple): GPS coordinates as (latitude, longitude, altitude).
    """
    print("Navigating to point:", gps_point)

    latitude, longitude, altitude = gps_point
    print(f"Latitude: {latitude}, Longitude: {longitude}, Altitude: {altitude}")
    
    await drone.action.goto_location(latitude, longitude, altitude, 0)
    await asyncio.sleep(1)

    print("Finished navigation.")

async def navigate(gps_points, drones, timestamps = []):
    """
    Navigate the drones to the given list of GPS points based on timestamps.
    
    Args:
        gps_points (list of tuples): List of GPS coordinates as (latitude, longitude, altitude).
        timestamps (list of (float, int)): Corresponding timestamp (in seconds), drone-id pairs.
        drones (list of mavsdk.System): List of connected drones.
    """
    # For now, this is just a placeholder to show function signature
    print("Starting navigation...")
    for i, point in enumerate(gps_points):
        latitude, longitude, altitude = point
        
        print(f"Navigating drone to point {i}: {latitude}, {longitude}, {altitude} at time {time}")
        # Add the actual navigation logic here, e.g., sending commands to the drone
        drones[0] = await navigate(point, drones[0])
    
    print("Finished navigation.")

async def run():
    # Connect to the Gazebo simulated PX4
    drones = []
    drones.append(await connect_drone("udp://:14540"))
    
    for drone in drones:
        await arm_and_takeoff(drone)

    # drones.append(await connect_drone("udp://:14541"))
    
    # Example usage of the navigate function
    gps_points = [(47.3977415, 8.5455939, 10), (47.3982415, 8.5465939, 10)]
    timestamps = [0, 5]
    
    await navigate(gps_points[0], drones[0])
    # await navigate(gps_points, drones[0])
    # await navigate(gps_points, drones, timestamps)

if __name__ == "__main__":
    loop = asyncio.get_event_loop()
    print("Here")
    loop.run_until_complete(run())