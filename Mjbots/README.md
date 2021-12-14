# Mjbots components

https://mjbots.com/

# Motors
https://mjbots.com/products/qdd100-beta-2

# controllers (moteus r4-5)
https://mjbots.com/products/moteus-r4-5

# power board
https://mjbots.com/products/mjbots-power-dist-r4-3b

# STORY

Was really impatient to receive my "pack"
Big Big thanks to Josh PIEPER for it patience, and help, to finalize a difficult order lol.
![Alt text](../images/mjbots/box1.jpg?raw=true)
![Alt text](../images/mjbots/box2.jpg?raw=true)
![Alt text](../images/mjbots/box3.jpg?raw=true)
![Alt text](../images/mjbots/box4.jpg?raw=true)

# Some informations :

* Install moteus_gui

        sudo apt-get install python3.8 python3.8-dev
        sudo apt install python3-pip
        sudo apt install python3-pyside2* python3-serial python3-can python3-matplotlib python3-qtconsole
        sudo pip3 install asyncqt importlib_metadata pyelftools cython
        python3.8 -m pip install moteus_gui
        
* SocketCan timings:

        sudo ip link set can0 up type can tq 12 prop-seg 25 phase-seg1 25 phase-seg2 29 sjw 10 dtq 12 dprop-seg 6 dphase-seg1 2 dphase-seg2 7 dsjw 12 restart-ms 1000 fd on

* bash test :

        for ID in $(seq 1 12); do echo "d pos nan 0 5" | moteus_tool -t $ID -c; done

## CAN-FD frame exemple ##

A single CAN-FD frame can be used to command the servo, and initiate a
query of certain registers.  An example frame might look like the
following, encoded in hex with annotations.

- `01` - write a single int8 register (number of registers is encoded
  in the 2 LSBs)
 - `00` - start register number "Mode"
 - `0a` - "position" mode
- `07` - write 3x int16 registers (number of registers is encoded in
  the 2 LSBs)
 - `20` - register 0x020
 - `6000` - position = 0x0060 = 96 = 3.456 degrees
 - `2001` - velocity = 0x0120 = 288 = 25.92 dps
 - `50ff` - feedforward torque = 0xff50 = -176 = 1.76 N*m
- `14` - read int16 registers
 - `04` - read 4 registers
 - `00` - starting at 0x000 (so 0x000 Mode, 0x001 Position, 0x002
   Velocity, 0x003 Torque)
- `13` - read 3x int8 registers
 - `0d` - starting at 0x00d (so 0x00d Voltage, 0x00e Temperature,
    0x00f Fault code)

Thus the whole CAN-FD message would be (in hex):

`01000a07206000200150ff140400130d`

To send this using the fdcanusb converter to a device configured at
the default address of 1, you could write.

`can send 8001 01000a07206000200150ff140400130d`

The `80` in ID is used for two purposes.  The high bit being set
forces the device to respond (otherwise it will not respond, even if
query commands are sent).  The remaining bits are the "ID" to respond
to.  In response to this command, a possible response from the servo
would look like:

`rcv 100 2404000a005000000170ff230d181400`

Decoded, that means:

- `100` from device "1" to device "0"

- `24` reply with int16 values
 - `04` 4 registers
 - `00` starting at register 0
 - `0a00` in mode 10 - Position
 - `5000` position is 0x0050 = 80 = 2.88 degrees
 - `0001` velocity is 0x0100 = 256 = 23.04 dps
 - `70ff` torque is 0xff70 = -144 = -1.44 Nm
- `23` reply with 3 int8 values
 - `0d` starting at register 0x00d
 - `18` voltage is 24V
 - `14` temperature is 20C
 - `00` no fault
