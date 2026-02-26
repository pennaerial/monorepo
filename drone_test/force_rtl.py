import asyncio
from mavsdk import System
from mavsdk.action import ActionError

# YOUR SPECIFIC DEVICE PATH
SERIAL_PORT = "/dev/tty.usbserial-DU0D9G4R"
BAUD_RATE = 57600

async def run():
    drone = System()
    print(f"--- Connecting via {SERIAL_PORT} ---")

    # Connect using the serial protocol
    await drone.connect(system_address=f"serial://{SERIAL_PORT}:{BAUD_RATE}")

    print("Waiting for drone to connect...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print("Successfully connected to Pixhawk!")
            break

    # Monitor flight mode to confirm transition
    print("Requesting Return-to-Launch (RTL)...")
    try:
        await drone.action.return_to_launch()
        print("Command sent. Verifying flight mode...")

        # Check the flight mode for 5 seconds to confirm
        async for flight_mode in drone.telemetry.flight_mode():
            print(f"Current Flight Mode: {flight_mode}")
            if str(flight_mode) == "RTL" or str(flight_mode) == "RETURN":
                print("SUCCESS: Drone is now in RTL mode.")
                break

    except ActionError as e:
        print(f"Failed to RTL: {e}")
    except Exception as e:
        print(f"An unexpected error occurred: {e}")

if __name__ == "__main__":
    asyncio.run(run())