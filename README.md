# ylo-2
New quadruped robot, working on UP Xtreme board, and BLDC motors from Mjbots
![Alt text](/IMG_20210207_131853.jpg?raw=true "Ylo-2")
![Alt text](/IMG_20210213_200148.jpg?raw=true "Ylo-2_and_my_son")

# XRDP on UBUNTU 18.04
curl -sL https://gist.github.com/djravine/88f2b9957a0bef6a6dd4c55aca951a09/raw | bash -s --

# ACTIVATE PEAK CAN3
sudo ip link set can3 up type can   tq 12 prop-seg 25 phase-seg1 25 phase-seg2 29 sjw 10   dtq 12 dprop-seg 6 dphase-seg1 2 dphase-seg2 7 dsjw 12   restart-ms 1000 fd on

# DESACTIVATE PEAK CAN3
sudo ip link set can3 down

# ACCESS MOTEUS UNDER TVIEW, VIA PEAK BOARD
python3.8 -m moteus_gui.tview --devices=11 --can-iface socketcan --can-chan can3

