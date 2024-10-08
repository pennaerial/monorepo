import asyncio
from mavsdk import System


async def navigate(gps_points, timestamps, drones):
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
        time, drone_id = timestamps[i]
        drone = drones[drone_id]
        print(f"Navigating drone {drone_id} to point {i}: {latitude}, {longitude}, {altitude} at time {time}")

        await drone.action.goto_location(latitude, longitude, altitude, 0)


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

    print("Arming drone...")
    await drone.action.arm()

    print("Taking off...")
    await drone.action.set_takeoff_altitude(5)
    await drone.action.takeoff()
    await asyncio.sleep(30)
    print("Taking off complete.")
    #example use
    gps_points = [(47.3977415, 8.5455939, 10), (47.3982415, 8.5465939, 10)]
    timestamps = [(0, 0), (5, 0)]
    drones = [drone]



    await navigate(gps_points, timestamps, drone)

    await asyncio.sleep(10)
    print("Landing...")
    await drone.action.land()


if __name__ == "__main__":
    loop = asyncio.get_event_loop()
    print("Here")
    loop.run_until_complete(run())