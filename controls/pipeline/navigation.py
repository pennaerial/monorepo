import asyncio
from mavsdk import System

async def navigate(drone, gps_points, timestamps):
    """
    Navigate the drone to the given list of GPS points based on timestamps.
    
    Args:
        gps_points (list of tuples): List of GPS coordinates as (latitude, longitude, altitude).
        timestamps (list of float): Corresponding timestamps in seconds.
    """

    start_time = asyncio.get_event_loop().time()

    # For now, this is just a placeholder to show function signature
    print("Starting navigation...")
    for i, (point, time) in enumerate(zip(gps_points, timestamps)):
        latitude, longitude, altitude = point
        current_time = asyncio.get_event_loop().time() - start_time
        wait_time = max(0, time - current_time)

        print(f"Navigating to point {i}: {latitude}, {longitude}, {altitude} at time {time}")
        # Add the actual navigation logic here, e.g., sending commands to the drone
        if wait_time:
            await asyncio.sleep(wait_time)
        await drone.action.goto_location(latitude, longitude, altitude, 0)
        await asyncio.sleep(3)

    print("Finished navigation.")

async def run():
    # Connect to the Gazebo simulated PX4
    drone = System()
    await drone.connect(system_address="udp://:14540")
    print("Waiting for drone to connect...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print("Drone connected!")
            break
    
    # Example usage of the navigate function
    gps_points = [(47.3977415, 8.5455939, 10), (47.3982415, 8.5465939, 10)]
    timestamps = [0, 5]
    
    await navigate(drone, gps_points, timestamps)

if __name__ == "__main__":
    loop = asyncio.get_event_loop()
    print("Here")
    loop.run_until_complete(run())