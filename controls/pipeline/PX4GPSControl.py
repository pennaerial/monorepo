import asyncio
from mavsdk import System
# async def run():
#     # Connect to the drone via UDP port 14540
#     drone = System()
    
#     await drone.connect(system_address= "udp://127.0.0.1:14280")
#     print("Waiting for drone to connect...")
#     async for state in drone.core.connection_state():
#         if state.is_connected:
#             print("Drone connected!")
#             break
#     print("Arming the drone...")
#     await drone.action.arm()
#     print("Taking off...")
#     await drone.action.takeoff()
#     # Wait for a few seconds
#     await asyncio.sleep(10)
#     print("Landing the drone...")
#     await drone.action.land()

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

async def run():
    # Connect to the Gazebo simulated PX4
    drone = System()
    await drone.connect(system_address="udp://10.102.218.92:14580")
    print("Waiting for drone to connect...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print("Drone connected!")
            break
    print("Arming the drone...")
    await drone.action.arm()
    print("Taking off...")
    await drone.action.takeoff()
    # Wait for the drone to stabilize
    await asyncio.sleep(5)
    # Get the current position
    async for position in drone.telemetry.position():
        current_lat = position.latitude_deg
        current_lon = position.longitude_deg
        altitude = position.absolute_altitude_m
        break  # Break the loop after getting the first position
    # Get the current heading (yaw) of the drone
    async for attitude in drone.telemetry.heading():
        heading = attitude.heading_deg
        break  # Break the loop after getting the first attitude
    # Calculate new latitude and longitude for 10 feet forward
    new_lat, new_lon = calculate_new_position(current_lat, current_lon, heading, 10)
    print(f"Moving to new location: Latitude: {new_lat}, Longitude: {new_lon}")
    # Move the drone to the new calculated location
    await drone.action.goto_location(new_lat, new_lon, altitude + updown , heading)
    # Wait for a few seconds to allow the drone to move
    await asyncio.sleep(10)
    print("Landing the drone...")
    await drone.action.land()

if __name__ == "__main__":
    loop = asyncio.get_event_loop()
    print("Here")
    loop.run_until_complete(run())