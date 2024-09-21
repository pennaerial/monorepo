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
    await asyncio.sleep(10)

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
    await asyncio.sleep(20)

    print("Finished navigation.")
    return drone

async def run():
    # Connect to the drones
    drones = []
    drones.append(await connect_drone("udp://10.102.218.92:14580"))
    
    # Arm and takeoff the drone
    for drone in drones:
        await arm_and_takeoff(drone)
    
    # Example usage of the navigate function
    gps_points = [(47.3977415, 8.5455939, 10), (47.3982415, 8.5465939, 10)]
    timestamps = [0, 5]
    
    drones[0] = await navigate(gps_points[0], drones[0])
    # await navigate(gps_points, drones[0])
    # await navigate(gps_points, drones, timestamps)
    

    #land all drones
    for i in range(len(drones)):
        print("Landing drone", i)
        await drones[i].action.land()
        await asyncio.sleep(10)

if __name__ == "__main__":
    loop = asyncio.get_event_loop()
    print("Here")

    loop.run_until_complete(run())