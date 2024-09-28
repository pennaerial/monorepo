import asyncio
import math
from mavsdk import System


def calculate_new_position(lat, lon, heading, distance_in_feet):
    # Convert distance in feet to distance in latitude/longitude degrees
    distance_in_degrees_lat = distance_in_feet / 364000  # 1 degree latitude ≈ 364,000 feet
    # Longitude adjustment depends on latitude (closer to poles means less distance per degree)
    distance_in_degrees_lon = distance_in_feet / (math.cos(math.radians(lat)) * 288200)
    # Calculate new latitude and longitude based on the heading (assuming forward is along the heading)
    new_lat = lat + distance_in_degrees_lat * math.cos(math.radians(heading))
    new_lon = lon + distance_in_degrees_lon * math.sin(math.radians(heading))
    return new_lat, new_lon

# Task: Takes in distance in feet and directions and a set of directions
# Directions: forward, backward, left, right, up, down, cw, ccw
async def navigate_set_directions(directions, address):
    drone = System()
    await drone.connect(system_address=address)

    async for state in drone.core.connection_state():
        if state.is_connected:
            print("Drone connected!")
            break
    print("Arming the drone...")
    await drone.action.arm()
    print("Taking off...")
    await drone.action.takeoff()

    await asyncio.sleep(10)
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

    for i in range(len(directions)):
        direction = directions[i][0]
        distance = directions[i][1]
        if direction == 'forward':
            new_lat, new_lon = calculate_new_position(current_lat, current_lon, heading, distance)
            print('going forward')
        if direction == 'backward':
            new_lat, new_lon = calculate_new_position(current_lat, current_lon, -heading, distance)
            print('going backward')
        if direction == 'left':
            new_lat, new_lon = calculate_new_position(current_lat, current_lon, heading + 90, distance)
            print('going left')
        if direction == 'right':
            new_lat, new_lon = calculate_new_position(current_lat, current_lon, heading - 90, distance)
            print('going right')
        if direction == 'up':
            altitude += distance
            print('going up')
        if direction == 'down':
            altitude -= distance
            print('going down')
        if direction == 'cw':
            heading += distance
            print('turning clockwise')
        if direction == 'ccw':
            heading -= distance
            print('going counterclockwise')
        await drone.action.goto_location(new_lat, new_lon, altitude, heading)
        print(new_lat, new_lon, altitude)
        current_lat = new_lat
        current_lon = new_lon
        await asyncio.sleep(5)
    await drone.action.land()


# navigate the thing in a circle
async def navigate_circle(radius, theta, address):
    drone = System()
    await drone.connect(system_address=address)

    async for state in drone.core.connection_state():
        if state.is_connected:
            print("Drone connected!")
            break
    print("Arming the drone...")
    await drone.action.arm()
    print("Taking off...")
    await drone.action.takeoff()

    await asyncio.sleep(10)
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

    current_heading = heading
    # suppose you break the circle into theta/360 parts
    while heading < (360 + heading):
        heading += theta
        # Turn
        await drone.action.go_to_location(current_lat, current_lon, altitude, heading)
        await asyncio.sleep(0.1)
        distance = 2 * radius * math.sin(math.radians(theta))
        new_lat, new_lon = calculate_new_position(current_lat, current_lon, heading, distance)
        # Move a little bit in the next direction
        await drone.action.goto_location(new_lat, new_lon, altitude, heading)
        await asyncio.sleep(1)

async def navigate_spiral_outwards(radius, theta, address):
    small_radius = radius/100

    drone = System()
    await drone.connect(system_address=address)

    async for state in drone.core.connection_state():
        if state.is_connected:
            print("Drone connected!")
            break
    print("Arming the drone...")
    await drone.action.arm()
    print("Taking off...")
    await drone.action.takeoff()

    await asyncio.sleep(10)
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

    while small_radius < radius:
        heading += theta
        # Turn
        await drone.action.go_to_location(current_lat, current_lon, altitude, heading)
        await asyncio.sleep(0.1)
        distance = 2 * small_radius * math.sin(math.radians(theta))
        new_lat, new_lon = calculate_new_position(current_lat, current_lon, heading, distance)
        # Move a little bit in the next direction
        await drone.action.goto_location(new_lat, new_lon, altitude, heading)
        await asyncio.sleep(1)
        current_lat, current_lon = new_lat, new_lon
        small_radius += radius/100

async def navigate_spiral_inwards(radius, theta, address):
    small_radius = radius/100

    drone = System()
    await drone.connect(system_address=address)

    async for state in drone.core.connection_state():
        if state.is_connected:
            print("Drone connected!")
            break
    print("Arming the drone...")
    await drone.action.arm()
    print("Taking off...")
    await drone.action.takeoff()

    await asyncio.sleep(10)
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

    while radius > small_radius:
        heading += theta
        # Turn
        await drone.action.go_to_location(current_lat, current_lon, altitude, heading)
        await asyncio.sleep(0.1)
        distance = 2 * radius * math.sin(math.radians(theta))
        new_lat, new_lon = calculate_new_position(current_lat, current_lon, heading, distance)
        # Move a little bit in the next direction
        await drone.action.goto_location(new_lat, new_lon, altitude, heading)
        await asyncio.sleep(1)
        current_lat, current_lon = new_lat, new_lon
        radius -= radius/100


async def navigate_set_directions_time(directions, timestamps, address):
    drone = System()
    await drone.connect(system_address=address)

    async for state in drone.core.connection_state():
        if state.is_connected:
            print("Drone connected!")
            break
    print("Arming the drone...")
    await drone.action.arm()
    print("Taking off...")
    await drone.action.takeoff()

    await asyncio.sleep(10)
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

    for i in range(1, len(directions)):
        direction = directions[i][0]
        distance = directions[i][1] / 3.281

        if direction == 'forward':
            new_lat, new_lon = calculate_new_position(current_lat, current_lon, heading, distance)
            print('going forward')
        if direction == 'backward':
            new_lat, new_lon = calculate_new_position(current_lat, current_lon, -heading, distance)
            print('going backward')
        if direction == 'left':
            new_lat, new_lon = calculate_new_position(current_lat, current_lon, heading + 90, distance)
            print('going left')
        if direction == 'right':
            new_lat, new_lon = calculate_new_position(current_lat, current_lon, heading - 90, distance)
            print('going right')
        if direction == 'up':
            altitude += distance
            print('going up')
        if direction == 'down':
            altitude -= distance
            print('going down')
        if direction == 'cw':
            heading += distance
            print('turning clockwise')
        if direction == 'ccw':
            heading -= distance
            print('going counterclockwise')
        speed = distance/(timestamps[i] - timestamps[i-1])
        await drone.set_current_speed(speed)
        await drone.goto_location(new_lat, new_lon)
        current_lat, current_lon = new_lat, new_lon

async def navigate_circle_time(radius, theta, address):
    pass

async def navigate_inward_spiral_time(radius, theta, address):
    pass

async def navigate_outward_spiral_time(radius, theta, adress):
    pass

async def navigate_single_point(gps_point, address):
    # connect to the drone
    drone = System()
    await drone.connect(system_address=address)

    await drone.action.arm()
    lat, long, alt = gps_point

    await drone.action.takeoff()
    await asyncio.sleep(10)

    async for attitude in drone.telemetry.heading():
        heading = attitude.heading_deg
        break  # Break the loop after getting the first attitude



    async for position in drone.telemetry.position():
        current_lat = position.latitude_deg
        current_lon = position.longitude_deg
        altitude = position.absolute_altitude_m
        break  # Break the loop after getting the first position

    await drone.action.goto_location(lat, long, altitude)
    pass

async def navigate_multiple_points(gps_points):
    pass

async def navigate(gps_points, timestamps):
    """
    Navigate the drone to the given list of GPS points based on timestamps.
    
    Args:
        gps_points (list of tuples): List of GPS coordinates as (latitude, longitude, altitude).
        timestamps (list of float): Corresponding timestamps in seconds.
    """
    # For now, this is just a placeholder to show function signature
    print("Starting navigation...")
    for i, point in enumerate(gps_points):
        latitude, longitude, altitude = point
        time = timestamps[i]
        print(f"Navigating to point {i}: {latitude}, {longitude}, {altitude} at time {time}")
        # Add the actual navigation logic here, e.g., sending commands to the drone
    
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

    await drone.action.takeoff()
    
    await asyncio.sleep(10)

    # Wait for a few seconds to allow the drone to move
    await asyncio.sleep(10)
    print("Landing the drone...")
    await drone.action.land()

    
    # Example usage of the navigate function
    gps_points = [(47.3977415, 8.5455939, 10), (47.3982415, 8.5465939, 10)]
    timestamps = [0, 5]
    
    await navigate(gps_points, timestamps)

if __name__ == "__main__":
    loop = asyncio.get_event_loop()
    print("Here")
    # loop.run_until_complete(navigate_set_directions([('forward', 10), ('left', 10), ('backward', 10), ('right', 10), ('cw', 90), ('forward', 10)], "udp://10.102.218.92:14580"))
    loop.run_until_complete(navigate_circle(10, "udp://10.102.218.92:14580", 2))