# Respeaker 4 microphones Array, 12 RGB leds, VAD...

https://wiki.seeedstudio.com/ReSpeaker_Mic_Array_v2.0/

The 12 RGB leds are programmables.

I'll use them to simulate brain activity, orders feedback, and humor.

![Alt text](../images/respeaker/overview.png?raw=true)

![Alt text](../images/respeaker/Pin_Map.png?raw=true)

----

### Here are the usb_pixel_ring APIs :

![Alt text](../images/respeaker/led_values.png?raw=true)
----

### Test leds :

    git clone https://github.com/respeaker/pixel_ring.git
    cd pixel_ring
    sudo python setup.py install
    sudo python examples/usb_mic_array.py

![Alt text](../images/respeaker/led_code.png?raw=true)

### VAD (Voice Activity Detection) :

* Step 1. Download the usb_4_mic_array.

        git clone https://github.com/respeaker/usb_4_mic_array.git
        cd usb_4_mic_array

* Step 2. Create a VAD.py with below code under usb_4_mic_array folder and run 'python VAD.py'

![Alt text](../images/respeaker/vad_code.png?raw=true)

----

# Errors

#### ImportError: No module named usb.core

    sudo pip install pyusb

# ROS-Support

https://github.com/furushchev/respeaker_ros