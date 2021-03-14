# ylo-2
New quadruped robot, working on UP Xtreme board, and BLDC motors from Mjbots
![Alt text](/IMG_20210207_131853.jpg?raw=true "Ylo-2")

![Alt text](/IIMG_20210213_200148.jpg?raw=true "Ylo-2")

# XRDP on UBUNTU 18.04
curl -sL https://gist.github.com/djravine/88f2b9957a0bef6a6dd4c55aca951a09/raw | bash -s --

# ACCESS MOTEUS UNDER TVIEW, VIA PEAK BOARD
python3.8 -m moteus_gui.tview --devices=11 --can-iface socketcan --can-chan can3

