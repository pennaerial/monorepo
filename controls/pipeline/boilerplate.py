import asyncio
from mavsdk import System
import math

async def connect_drone(system_address):
    drone = System(mavsdk_server_address=system_address)
    await drone.connect()
    async for state in drone.core.connection_state():
        if state.is_connected:
            return drone

async def navigate(drone, commands):
    """
    Navigate the drone to the given list of GPS points based on timestamps.
    
    Args:
        gps_points (list of tuples): List of GPS coordinates as (latitude, longitude, altitude).
        timestamps (list of float): Corresponding timestamps in seconds.
    """
    # For now, this is just a placeholder to show function signature
    print("Starting navigation...")

    async for pos in drone.telemetry.position():
        lat = pos.latitude_deg
        lon = pos.longitude_deg
        alt = pos.relative_altitude_m
        break  
    async for telem in drone.telemetry.attitude_euler():
        heading = telem.yaw_deg
        break

    gps_points = []
    for direction, distance_in_feet in commands:
        lat, lon, alt = calculate_new_position(lat, lon, heading, alt, direction, distance_in_feet)
        gps_points.append((lat, lon, alt))

    for i, point in enumerate(gps_points):
        latitude, longitude, altitude = point
        print(f"Navigating to point {i}: {latitude}, {longitude}, {altitude}")
        await drone.action.goto_location(latitude, longitude, altitude, 0)

    print("Finished navigation.")

async def run():
    drones = []
    await drones.append(await connect_drone("udp://10.102.218.92:14580"))
    commands = [("forward", 10), ("right", 10),  ("up", 10), ("backward", 10), ("left", 10), ("down", 10)]        
    await navigate(drones[0], commands)

def calculate_new_position(lat, lon, heading, alt, direction, distance_in_feet):
    distance_in_degrees_lat = distance_in_feet / 364000
    distance_in_degrees_lon = distance_in_feet / (math.cos(math.radians(lat)) * 288200)
    match direction:    
        case "forward":
            lat = lat + distance_in_degrees_lat * math.cos(math.radians(heading))
            lon = lon + distance_in_degrees_lon * math.sin(math.radians(heading))
        case "backward":
            lat = lat - distance_in_degrees_lat * math.cos(math.radians(heading))
            lon = lon - distance_in_degrees_lon * math.sin(math.radians(heading))
        case "left":
            lat = lat + distance_in_degrees_lat * math.cos(math.radians(heading + 90)) 
            lon = lon + distance_in_degrees_lon * math.sin(math.radians(heading + 90))
        case "right":
            lat = lat + distance_in_degrees_lat * math.cos(math.radians(heading - 90))
            lon = lon + distance_in_degrees_lon * math.sin(math.radians(heading - 90))
        case "up":
            alt = alt + distance_in_feet
        case "down":
            alt = alt - distance_in_feet
    return lat, lon, alt

if __name__ == "__main__":
    print("Here")
    asyncio.run(run())
    