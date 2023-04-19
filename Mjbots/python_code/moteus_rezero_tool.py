#!/usr/bin/env python3

# Copyright 2023. Vincent FOUCAULT elpimous12@gmail.com.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# -------------------------------------------------------------------------------

import os
import subprocess
import sys, time
import tempfile

MOTEUS_TOOL = 'python3.8 -m moteus.moteus_tool --can-iface pcan --can-chan '

# Eack PEAK_CAN port controls 3 motors
# ex : PCAN_PCIBUS1 -> 1-3 ; PCAN_PCIBUS2 -> 4-6...
def pcan_pcibusx(controller):
    PCAN_PCIBUSx = ''
    if int(controller) < 4 :
        PCAN_PCIBUSx = 'PCAN_PCIBUS1'
    elif 3 < int(controller) < 7 :
        PCAN_PCIBUSx = 'PCAN_PCIBUS3'
    elif 6 < int(controller) < 10 :
        PCAN_PCIBUSx = 'PCAN_PCIBUS4'
    else  :
        PCAN_PCIBUSx = 'PCAN_PCIBUS2'
    return PCAN_PCIBUSx


def main():


    print("\n\n               *********************************************************************")
    print("               *                                                                   *")
    print("               *                   Rezeroing Moteus motor !!!                      *")
    print("               *                                                                   *")
    print("               *********************************************************************\n\n")

    controller = input("enter Moteus motor ID to rezero : ")

    a = input("ok to rezero motor "+ controller+" ?")

    if a == "y":
        command = ("python3.8 -m moteus.moteus_tool -t " + controller + " --can-iface pcan --can-chan "+pcan_pcibusx(controller)+" --zero-offset")
        # python3 -m moteus.moteus_tool --target 1 --zero-offset 
        os.system(command)
        print(" ---> Rezero motor "+controller+" on port "+pcan_pcibusx(controller)+" Done !!!")
    else:
        print("Exiting...")
        pass

main()