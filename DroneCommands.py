import asyncio
import datetime
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

    for i in range(0, len(directions)):
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
        if (i == 0):
            speed = distance/(timestamps[i])
        else:
            speed = distance/(timestamps[i] - timestamps[i-1])
        await drone.set_current_speed(speed)
        await drone.goto_location(new_lat, new_lon)
        current_lat, current_lon = new_lat, new_lon

async def navigate_circle_time(radius, theta, time, address):
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

    start_time = datetime.now()

    while ((datetime.now() - start_time).total_seconds() < time) and heading < (360 + heading):
        heading += theta
        # Turn
        await drone.action.go_to_location(current_lat, current_lon, altitude, heading)
        await asyncio.sleep(0.1)

        # Calculate the new position based on distance and heading
        distance = 2 * radius * math.sin(math.radians(theta))
        new_lat, new_lon = calculate_new_position(current_lat, current_lon, heading, distance)
        
        # Move to the new location
        await drone.action.goto_location(new_lat, new_lon, altitude, heading)
        await asyncio.sleep(1)

async def navigate_inward_spiral_time(radius, theta, time, address):
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

    start_time = datetime.now()

    while radius > small_radius and ((datetime.now() - start_time) < time):
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

async def navigate_outward_spiral_time(radius, theta, time, address):
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

    start_time = datetime.now()

    while (datetime.now() - start_time) < time:
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

class PID:
    def __init__(self, Kp, Ki, Kd, setpoint=0):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.setpoint = setpoint
        self.integral = 0
        self.previous_error = 0

    def compute(self, current_value, dt):
        error = self.setpoint - current_value
        self.integral += error * dt
        derivative = (error - self.previous_error) / dt
        output = self.Kp * error + self.Ki * self.integral + self.Kd * derivative
        self.previous_error = error
        return output

async def navigate_interpolated_trajectory(directions, timestamps, address):
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

    # Get the initial position and heading of the drone
    async for position in drone.telemetry.position():
        current_lat = position.latitude_deg
        current_lon = position.longitude_deg
        altitude = position.absolute_altitude_m
        break
    
    async for attitude in drone.telemetry.heading():
        heading = attitude.heading_deg
        break

    # Initialize PID controllers for latitude, longitude, and altitude control
    lat_pid = PID(Kp=1.0, Ki=0.1, Kd=0.05)
    lon_pid = PID(Kp=1.0, Ki=0.1, Kd=0.05)
    alt_pid = PID(Kp=1.0, Ki=0.1, Kd=0.05)

    for i in range(len(directions) - 1):
        start_time = timestamps[i]
        end_time = timestamps[i + 1]
        direction = directions[i]
        next_direction = directions[i + 1]
        
        # Calculate the distance and heading change between the current and next point
        distance = directions[i][1] / 3.281
        next_distance = directions[i + 1][1] / 3.281
        heading_change = next_direction[0] - direction[0]

        # Get the time step for interpolation
        total_time = end_time - start_time
        time_steps = 100  # Define number of steps between points for smooth interpolation
        dt = total_time / time_steps

        # Interpolate the trajectory between the current and next waypoint
        for step in range(time_steps):
            progress = step / time_steps
            interpolated_lat = current_lat + progress * (next_direction[0] - current_lat)
            interpolated_lon = current_lon + progress * (next_direction[1] - current_lon)
            interpolated_alt = altitude + progress * (next_direction[2] - altitude)

            # Calculate PID adjustments based on the errors
            lat_correction = lat_pid.compute(interpolated_lat, dt)
            lon_correction = lon_pid.compute(interpolated_lon, dt)
            alt_correction = alt_pid.compute(interpolated_alt, dt)

            # Apply the corrections and move the drone
            new_lat = current_lat + lat_correction
            new_lon = current_lon + lon_correction
            new_alt = altitude + alt_correction

            await drone.action.goto_location(new_lat, new_lon, new_alt)
            current_lat, current_lon, altitude = new_lat, new_lon, new_alt

            await asyncio.sleep(dt)

    print("Landing the drone...")
    await drone.action.land()



if __name__ == "__main__":
    loop = asyncio.get_event_loop()
    print("Here")
    # loop.run_until_complete(navigate_set_directions([('forward', 10), ('left', 10), ('backward', 10), ('right', 10), ('cw', 90), ('forward', 10)], "udp://10.102.218.92:14580"))
    loop.run_until_complete(navigate_circle(10, "udp://10.102.218.92:14580", 2))