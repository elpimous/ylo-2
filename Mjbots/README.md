# Mjbots components

https://mjbots.com/

# Motors
https://mjbots.com/products/qdd100-beta-2

![Alt text](../images/mjbots/qdd100.jpg?raw=true)

# controllers (moteus r4-5)
https://mjbots.com/products/moteus-r4-5

![Alt text](../images/mjbots/moteus-r45.jpg?raw=true)

# power board
https://mjbots.com/products/mjbots-power-dist-r4-3b

![Alt text](../images/mjbots/power_dist_r43b.jpg?raw=true)


# STORY

Was really impatient to receive my "pack"
Big Big thanks to Josh PIEPER for it patience, and help, to finalize a difficult order lol.
![Alt text](../images/mjbots/box1.jpg?raw=true)
![Alt text](../images/mjbots/box2.jpg?raw=true)
![Alt text](../images/mjbots/box3.jpg?raw=true)
![Alt text](../images/mjbots/box4.jpg?raw=true)

# Some informations :

* SocketCan timings:

        ip link set can0 up type can \
        tq 12 prop-seg 25 phase-seg1 25 phase-seg2 29 sjw 10 \
        dtq 12 dprop-seg 6 dphase-seg1 2 dphase-seg2 7 dsjw 12 \
        restart-ms 1000 fd on

* bash test :

        for ID in $(seq 1 12); do echo "d pos nan 0 5" | moteus_tool -t $ID -c; done