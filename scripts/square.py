import asyncio
from mavsdk import System
from mavsdk.offboard import (OffboardError, VelocityBodyYawspeed)

# Rate at which we will send setpoint commands
COMMAND_RATE = 10  # 10 Hz

async def print_velocity(drone):
    """Prints the drone's velocity in the NED frame."""
    try:
        async for velocity in drone.telemetry.velocity_ned():
            print(
                f"Velocity NED: "
                f"N: {velocity.north_m_s:+.2f} m/s, "
                f"E: {velocity.east_m_s:+.2f} m/s, "
                f"D: {velocity.down_m_s:+.2f} m/s"
            )
            await asyncio.sleep(1)
    except asyncio.CancelledError:
        print("Velocity printing task stopped.")

async def send_velocity_commands_for_duration(drone, forward, right, down, yaw, duration_s):
    """
    Sends velocity commands in a loop for a specific duration.
    """
    print(f"Sending velocity command: F:{forward}, R:{right}, D:{down} for {duration_s}s")
    command = VelocityBodyYawspeed(forward, right, down, yaw)
    
    # Send commands for the specified duration
    for _ in range(duration_s * COMMAND_RATE):
        await drone.offboard.set_velocity_body(command)
        await asyncio.sleep(1 / COMMAND_RATE)


async def run():
    """Main function to run the mission."""
    drone = System()
    await drone.connect(system_address="udp://:14540")

    print("Waiting for drone to connect...")
    async for state in drone.telemetry.flight_mode():
        if state is not None:
            print("Drone discovered!")
            break

    velocity_task = asyncio.create_task(print_velocity(drone))

    try:
        print("Waiting for drone to have a global position estimate...")
        async for health in drone.telemetry.health():
            if health.is_global_position_ok and health.is_home_position_ok:
                print("Global position estimate OK")
                break

        print("-- Arming")
        await drone.action.arm()

        print("-- Taking off")
        await drone.action.set_takeoff_altitude(5.0)
        await drone.action.takeoff()
        await asyncio.sleep(10)

        print("-- Starting Offboard")
        # Send a dummy command before starting offboard mode
        await drone.offboard.set_velocity_body(VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0))
        await drone.offboard.start()

        print("-- Flying square pattern by streaming setpoints")

        # Fly forward
        await send_velocity_commands_for_duration(drone, 5.0, 0.0, 0.0, 0.0, 5)
        # Fly right
        await send_velocity_commands_for_duration(drone, 0.0, 5.0, 0.0, 0.0, 5)
        # Fly backward
        await send_velocity_commands_for_duration(drone, -5.0, 0.0, 0.0, 0.0, 5)
        # Fly left
        await send_velocity_commands_for_duration(drone, 0.0, -5.0, 0.0, 0.0, 5)
        
        # Stop
        print("-- Stopping")
        await send_velocity_commands_for_duration(drone, 0.0, 0.0, 0.0, 0.0, 2)


        print("-- Stopping offboard")
        await drone.offboard.stop()

        print("-- Landing")
        await drone.action.land()

    except Exception as e:
        print(f"An error occurred: {e}")
    finally:
        print("Mission finished. Stopping velocity printing task...")
        velocity_task.cancel()
        await asyncio.sleep(1)


if __name__ == "__main__":
    asyncio.run(run())