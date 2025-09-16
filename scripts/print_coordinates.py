import asyncio
from mavsdk import System

async def print_coordinates(drone):
    """Prints the drone's coordinates (latitude, longitude, altitude)."""
    try:
        async for position in drone.telemetry.position():
            print(
                f"Coordinates: "
                f"Lat: {position.latitude_deg:.6f}°, "
                f"Lon: {position.longitude_deg:.6f}°, "
                f"Alt: {position.absolute_altitude_m:.2f} m"
            )
            await asyncio.sleep(1)
    except asyncio.CancelledError:
        print("Coordinate printing task stopped.")

async def run():
    drone = System()
    await drone.connect(system_address="udp://:14540")
    coordinate_task = asyncio.create_task(print_coordinates(drone))
    await coordinate_task

asyncio.run(run())
