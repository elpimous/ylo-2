#!/usr/bin/env python3

# power_module.py

# YLo-2 Power module

# looks for power button status,

# asks for a soft shutdown, wait for a time, and powers off

# if error : RuntimeError: Unexpected schema announce for 'power' : 'b'emit power''
# the tview tree wasn't closed before exiting

# -------------------------------------------------------------------------------

import asyncio
import moteus
import os
import subprocess

pwd = 'change for your password !!!'

async def main():

    transport   = moteus.PythonCan(interface='pcan', channel='PCAN_PCIBUS3')

    c = moteus.Controller(id = 32, transport = transport)
 
    while True:

        stream = moteus.Stream(c)
        await stream.flush_read()

        # reset Lock time (20s)
        await stream.command("p lock 200".encode('latin1'))
        print("reset")
        # check power button state
        power_button = await stream.read_data("power")
        switch_status = power_button.switch_status

        print(switch_status)
        # switch state is off
        if switch_status == 0:
            # request a soft ubuntu shutdown
            break

        await asyncio.sleep(0.5)

    print("shutdown needed")
    os.system('echo '+pwd+' | sudo -S poweroff')
    print("command sent")
    # the program exits, Lock time decreases (by hardware)
    # and when finished, the power board cuts power.

if __name__ == '__main__':
    asyncio.run(main())