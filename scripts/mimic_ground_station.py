import asyncio
from mavsdk import System


async def run():
    drone = System()
    await drone.connect(system_address="udp://:14540")
    
    while True:
        await asyncio.sleep(1)


asyncio.run(run())
