#!/usr/bin/env python3

# Copyright 2020 Josh Pieper, jjp@pobox.com.
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

# modified : 07 june 2021
#----------------------------
# Set servo-level configuration for Ylo-2 robot.
# Using PEAK 4 can M2 board, and Moteus_tool.py

# MEMO:
# python3.8 -m moteus.moteus_tool -t 32 -c --can-iface pcan --can-chan PCAN_PCIBUS3
# echo "conf set servo.flux_brake_min_voltage 20.5" | python3.8 -m moteus.moteus_tool --can-iface pcan --can-chan PCAN_PCIBUS1 -t 2 --console
# You can also construct an arbitrary file with many commands ot send to --write-config

# -------------------------------------------------------------------------------

import os
import subprocess
import sys, time
import tempfile

SCRIPT_PATH = os.path.dirname(__file__)

MOTEUS_TOOL = 'python3.8 -m moteus.moteus_tool --can-iface pcan --can-chan '

controllers_1346791012 = ('1','3','4','6','7','9','10','12') 
controllers_25811      = ('2','5','8','11') # lower legs controllers

# general config
CONFIG_1346791012 = {
    'servopos.position_min' : '-.70',
    'servopos.position_max' : '.70',
    'servo.pwm_min' : '0.006',
    'servo.flux_brake_min_voltage' : '20.5',
    'servo.flux_brake_resistance_ohm' : '0.05',
    'servo.pid_position.kp' : '50',
    'servo.feedforward_scale' : '0.0',
    'servo.pid_position.kd' : '6',
    'servo.pid_dq.ki' : '150.0',
    'motor.unwrapped_position_scale' : '0.16666667',
    'drv8323_conf.idrivep_hs_ma' : '370',
    'drv8323_conf.idriven_hs_ma' : '740',
    'drv8323_conf.idrivep_ls_ma' : '370',
    'drv8323_conf.idriven_ls_ma' : '740',
    'drv8323_conf.dead_time_ns' : '50',
}

# specific config for 4 controllers
CONFIG_25811 = {
    'servopos.position_min' : '-.25',
    'servopos.position_max' : '.25',
    'servo.pwm_min' : '0.006',
    'servo.flux_brake_min_voltage' : '20.5',
    'servo.flux_brake_resistance_ohm' : '0.05',
    'servo.pid_position.kp' : '200',
    'servo.feedforward_scale' : '0.0',
    'servo.pid_position.kd' : '6',
    'servo.pid_dq.ki' : '150.0',
    'motor.unwrapped_position_scale' : '0.093750',
    'drv8323_conf.idrivep_hs_ma' : '370',
    'drv8323_conf.idriven_hs_ma' : '740',
    'drv8323_conf.idrivep_ls_ma' : '370',
    'drv8323_conf.idriven_ls_ma' : '740',
    'drv8323_conf.dead_time_ns' : '50',
}
    
# Eack PEAK_CAN port controls 3 motors
# ex : PCAN_PCIBUS1 -> 1-3 ; PCAN_PCIBUS2 -> 4-6...
def pcan_pcibusx(controller):
    PCAN_PCIBUSx = ''
    if int(controller) < 4 :
        PCAN_PCIBUSx = 'PCAN_PCIBUS1'
    elif 3 < int(controller) < 7 :
        PCAN_PCIBUSx = 'PCAN_PCIBUS2'
    elif 6 < int(controller) < 10 :
        PCAN_PCIBUSx = 'PCAN_PCIBUS3'
    else  :
        PCAN_PCIBUSx = 'PCAN_PCIBUS4'
    return PCAN_PCIBUSx


def main():
    #if os.geteuid() != 0:
    #    raise RuntimeError('This must be run as root')


    print("\n\n               ***************************************************************************************")
    print("               *                                                                                     *")
    print("               *                   WRITE PARAMS TO ALL 12 CONTROLLERS                                *")
    print("               *                                for Ylo-2 quadruped robot.                           *")
    print("               *                                                                                     *")
    print("               ***************************************************************************************\n\n")

    # loop for the 8 controllers
    for controller in controllers_1346791012:

        # open temporary config file to write...
        with tempfile.NamedTemporaryFile(delete=True) as config:

            # read and write each param to this file
            for key, value in CONFIG_1346791012.items():
                config.write('conf set {} {}\n'.format(key, value).encode('utf8'))
            config.flush()

            print("\nSend params to controller {}".format(controller))
            print('     command -> ', MOTEUS_TOOL + pcan_pcibusx(controller) + ' -t{}'.format(controller), '--write-config', config.name)
            os.system(MOTEUS_TOOL + pcan_pcibusx(controller) + ' -t{}'.format(controller), '--write-config', config.name)

        print("store them all persistently")
        print('     command -> echo "conf save" | python3.8 -m moteus.moteus_tool --can-iface pcan --can-chan ' + pcan_pcibusx(controller) + ' -t{} --console'.format(controller))
        run('echo "conf save" | python3.8 -m moteus.moteus_tool --can-iface pcan --can-chan ' + pcan_pcibusx(controller) + ' -t{} --console'.format(controller))

        print("\n     *** controller {} done ***".format(controller))
    

    # loop for the last 4 controllers
    for controller in controllers_25811:

        # open temporary config file to write...
        with tempfile.NamedTemporaryFile(delete=True) as config:

            # read and write each param to this file
            for key, value in CONFIG_25811.items():
                config.write('conf set {} {}\n'.format(key, value).encode('utf8'))
            config.flush()

            print("\nSend params to controller {}".format(controller))
            print('     command -> ', MOTEUS_TOOL + pcan_pcibusx(controller) + ' -t{}'.format(controller), '--write-config', config.name)
            os.system(MOTEUS_TOOL + pcan_pcibusx(controller) + ' -t{}'.format(controller), '--write-config', config.name)

        print("store them all persistently")
        print('     command -> echo "conf save" | python3.8 -m moteus.moteus_tool --can-iface pcan --can-chan ' + pcan_pcibusx(controller) + ' -t{} --console'.format(controller))
        run('echo "conf save" | python3.8 -m moteus.moteus_tool --can-iface pcan --can-chan ' + pcan_pcibusx(controller) + ' -t{} --console'.format(controller))

        print("\n     *** controller {} done ***".format(controller))
        

if __name__ == '__main__':
    main()
