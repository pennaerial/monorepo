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

import math
# Helper function to move drone forward by a certain distance
def calculate_new_position(lat, lon, heading, distance_in_feet):
    # Convert distance in feet to distance in latitude/longitude degrees
    distance_in_degrees_lat = distance_in_feet / 364000  # 1 degree latitude ≈ 364,000 feet
    # Longitude adjustment depends on latitude (closer to poles means less distance per degree)
    distance_in_degrees_lon = distance_in_feet / (math.cos(math.radians(lat)) * 288200)
    # Calculate new latitude and longitude based on the heading (assuming forward is along the heading)
    new_lat = lat + distance_in_degrees_lat * math.cos(math.radians(heading))
    new_lon = lon + distance_in_degrees_lon * math.sin(math.radians(heading))
    return new_lat, new_lon

def calculate_new_heading(heading, degrees):
    # turn the drone 
    return heading + degrees

async def get_current_position(drone):
    # Get the current position
    async for position in drone.telemetry.position():
        current_lat = position.latitude_deg
        current_lon = position.longitude_deg
        altitude = position.absolute_altitude_m
        break  # Break the loop after getting the first position


    # Get the current heading (yaw) of the drone
    async for attitude in drone.telemetry.heading():
        curr_heading = attitude.heading_deg
        break  # Break the loop after getting the first attitude

    print(f"Current Latitude: {current_lat}, Current Longitude: {current_lon}, Altitude: {altitude}, Heading: {curr_heading}")

    return current_lat, current_lon, altitude, curr_heading

async def navigate(directions, drone):
    """
    Navigate the drone in the given direction the given distance.
    
    Args:
        directions: [direction, distance]
    """

    dir, distance = directions
    move_heading = 0
    updown = 0
    res_heading = 0

    current_lat, current_lon, altitude, curr_heading = await get_current_position(drone)
    new_lat, new_lon = current_lat, current_lon

    if dir == "forward":
        move_heading = 0
    elif dir == "backward":
        move_heading = 180
    elif dir == "left":
        move_heading = 270
    elif dir == "right":
        move_heading = 90
    elif dir == "up":
        updown = 1
    elif dir == "down":
        updown = -1
    elif dir == "cw":
        res_heading = 90
    elif dir == "ccw":
        res_heading = -90
    
    if dir == "forward" or dir == "backward" or dir == "left" or dir == "right":
        new_lat, new_lon = calculate_new_position(current_lat, current_lon, curr_heading + move_heading, distance)
     
    new_altitude = altitude + updown * distance
    
    print(f"Moving to new location: Latitude: {new_lat}, Longitude: {new_lon}, Altitude: {new_altitude} Move Heading: {curr_heading + res_heading} New Heading: {curr_heading + res_heading}")
    # Move the drone to the new calculated location
    await drone.action.goto_location(new_lat, new_lon, new_altitude, curr_heading + res_heading)

    # Wait for a few seconds to allow the drone to move
    await asyncio.sleep(5)

    print("Finished navigation.")
    return drone

async def navigate_square(side, turn, drones):
    commands = []
    for _ in range(4):
        commands.append(["forward", side])
        commands.append(["cw", turn])
    
    for j in range(len(commands)):
        print(j, commands[j])
        for i in range(len(drones)):
            await navigate(commands[j], drones[i])
            await asyncio.sleep(5)
            

async def navigate_circle(radius, drones):
    segments = 36
    commands = []
    arc_length = 2 * math.pi * radius / segments

    for i in range(segments):
        commands.append(["forward", arc_length])
        commands.append(["cw", 360/segments])

    for j in range(len(commands)):
        print(j, commands[j])
        for i in range(len(drones)):
            await navigate(commands[j], drones[i])
            await asyncio.sleep(1)

async def run():
    # Connect to the Gazebo simulated PX4
    drones = []
    drones.append(await connect_drone("udp://10.102.218.92:14580"))
    
    for drone in drones:
        await arm_and_takeoff(drone)
    
    # go in a square
    await navigate_square(10, 90, drones)

    # go in a circle
    # await navigate_circle(10, drones)
    
    #land all drones
    for i in range(len(drones)):
        print("Landing drone", i)
        await drones[i].action.land()
        await asyncio.sleep(10)
    
    print("Finished running.")

if __name__ == "__main__":
    loop = asyncio.get_event_loop()
    print("Here")
    loop.run_until_complete(run())