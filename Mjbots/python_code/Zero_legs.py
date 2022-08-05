#!/usr/bin/python3 -B


import asyncio
import math, time
import moteus

class StandUp:
    def __init__(self):
        transport = moteus.PythonCan(interface='pcan', channel='PCAN_PCIBUS1')
        self.motor_2 = moteus.Controller(id=2, transport=transport)
        self.motor_5 = moteus.Controller(id=5, transport=transport)
        self.motor_6 = moteus.Controller(id=6, transport=transport)


    async def stop(self):
        print("relaxing motor")
        await self.motor_2.set_stop()
        print("done.")

    async def check_initial_pose(self):
        print("Analysing start pose")
        while True:
            await self.stop()
            await self.motor_2.set_rezero(0.0, query=True)
            result = None
            while result is None:
               result = await self.motor_2.query()
            if abs(result.values[moteus.Register.POSITION]) <= 0.5:
                break
        await self.standing()

    async def standing(self):
        print("standing...")
        await self.stop()
        await self.motor_2.set_position(
                position=math.nan,
                velocity=0.11,
                maximum_torque=3.0,
                stop_position=0.25,
                watchdog_timeout=math.nan,
                feedforward_torque=0.0,
                query=True)
        await asyncio.sleep(5.0) # needed, otherwise movs will stop
        
        await self.stop()

async def main():
    app = StandUp()
    await app.standing()
    #await app.check_initial_pose()

asyncio.run(main())
