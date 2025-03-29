R_earth = 6378137.0  # Earth's radius in meters (WGS84)

pink = ((140, 155, 50), (170, 255, 255))
green = ((40, 155, 50), (85, 255, 255))
blue = ((85, 100, 100), (140, 255, 255))
vehicle_map = ['quadcopter', 'tiltrotor_vtol', 'fixed_wing', 'standard_vtol']

def camel_to_snake(name):
    import re
    # Convert CamelCase to snake_case.
    s1 = re.sub('(.)([A-Z][a-z]+)', r'\1_\2', name)
    return re.sub('([a-z0-9])([A-Z])', r'\1_\2', s1).lower()