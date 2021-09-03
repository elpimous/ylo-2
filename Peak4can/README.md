# PEAK can Board, with 4 can ports
----
# PCAN-M.2

Interface CAN et CAN FD pour M.2 (PCIe)

https://www.peak-system.com/PCAN-M-2.473.0.html?&L=2

![Alt text](../images/peak/PCAN-M.2_4K.jpg?raw=true)

----
# INSTALLATION

* last driver : Linux Driver 8.12.0
https://www.peak-system.com/Details.114+M525b77fd42c.0.html?&L=1

* PCAN-Basic API (Linux)
https://www.peak-system.com/quick/BasicLinux

* PCAN-M.2 manual
https://www.peak-system.com/produktcd/Pdf/English/PCAN-M.2_UserMan_eng.pdf


### Check: Are CAN drivers part of your Linux environment?

Thanks to Robin FROJD for it help !   https://github.com/raess1
Major content come from : https://github.com/raess1/K3lso-CAN-communication


    grep PEAK_ /boot/config-`uname -r`

### Check: Is the CAN device initialized?


    lsmod | grep ^peak
    ls -l /dev/pcan*

### How to install PCAN-View via repository

Download and install the following file peak-system.list from the PEAK-System website:

    wget -q http://www.peak-system.com/debian/dists/`lsb_release -cs`/peak-system.list -O- | sudo tee /etc/apt/sources.list.d/peak-system.list
Then, download and install the PEAK-System public key for apt-secure, so that the repository is trusted:

    wget -q http://www.peak-system.com/debian/peak-system-public-key.asc -O- | sudo apt-key add -

Install pcanview-ncurses and other dependencies:

    sudo apt-get update
    sudo apt-get install pcanview-ncurses
    sudo apt-get install libpopt-dev

Install PCAN-M.2 Driver for Linux Driver installation:

    cd && mkdir -p ~/src/peak && cd ~/src/peak
    wget https://www.peak-system.com/fileadmin/media/linux/files/peak-linux-driver-8.12.0.tar.gz
    tar -xzf peak-linux-driver-8.12.0.tar.gz # Untar the compressed tarball file
    cd peak-linux-driver-8.12.0/ 
    make -C driver PCI=PCI_SUPPORT PCIEC=PCIEC_SUPPORT DNG=NO_DONGLE_SUPPORT USB=NO_USB_SUPPORT ISA=NO_ISA_SUPPORT PCC=NO_PCCARD_SUPPORT
    sudo make -C driver install
    make -C lib && sudo make -C lib install
    make -C test && sudo make -C test install # test utilities (optional)
    sudo modprobe pcan # driver loading

Launch PCAN-View for testing CAN communication:

    pcanview

### uninstall PCAN-M.2 Driver for Linux (Ubuntu 18.04)

    cd ~/src/peak/peak-linux-driver-8.*/ 
    sudo make uninstall
    rmmod pcan
    sudo reboot

export some more properties of the device

    for f in /sys/class/pcan/pcanpcifd1/*; do [ -f $f ] && echo -n "`basename $f` = " && cat $f; done 

lspcan overview of the PC CAN interfaces. The "-i" option displays static properties of devices nodes. with –T –t –s –f refreshes the screen every second with a detailed view

    lspcan -T -t -i

    ylo2@ylo2-UP-WHL01:~/src/peak/peak-linux-driver-8.12.0$ lspcan -T -t -i
    dev name	port	irq	clock	btrs	bus
    [PCAN-M.2 0]
    |_ pcanpcifd0	CAN1	16	80MHz	500k+2M	CLOSED
    |_ pcanpcifd1	CAN2	16	80MHz	500k+2M	CLOSED
    |_ pcanpcifd2	CAN3	16	80MHz	500k+2M	CLOSED
    |_ pcanpcifd3	CAN4	16	80MHz	500k+2M	CLOSED
    
---

# Some tests

    pcanview

Select Pcan Channel (for example -CAN0 /dev/pcan0) and configure it with the following settings:

    Clock Frequency (Hz) = 80000000
    Bitrate (bps) Nominal = 1000000
    Bitrate (bps) Data = 5000000
    Sample point (x100) Nominal = 6375
    Sample point (x100) Data = 5625
    Sync Jump Width Nominal = 10
    Sync Jump Width Data = 12

### Test 1

To send a first test command to verify communication with moteus (Edit Transmit Message)

        ID: (hex) = 8001
        Len = 3
        Data: (hex) = 42 01 20
        Cycle Time: (ms) = 0
        [x] Paused 
    - Message Type select the follwing: 
        [x] Extended Frame
        [x] CAN FD
        [x] Bit Rate Swtich
        Press OK

Under the Tx CAN-ID you can see your created Messages. to send one. simply select one and press (space).

In the Rx CAN-ID. you should now get a respons from moteus under data table with the following:

    41 01 00

### Test 2

Create two new Transmit Messages. (Edit Transmit Message)

first messange: Set postion to 0 and turn on torque.

    ID: (hex) = 8001
    Len = 19
    Data: (hex) = 01 00 0a 0e 20 00 00 00 00 00 00 00 00 11 00 1f 01 13 0d
    Cycle Time: (ms) = 0
    [x] Paused 
    Message Type select the follwing: 
    [x] Extended Frame
    [x] CAN FD
    [x] Bit Rate Swtich
    Press OK

Second messange: turn of torque

    ID: (hex) = 8001
    Len = 3
    Data: (hex) = 01 00 00
    Cycle Time: (ms) = 0
    [x] Paused 
    Message Type select the follwing: 
    [x] Extended Frame
    [x] CAN FD
    [x] Bit Rate Swtich
    Press OK

Send first messange and you should here the motor has applied torque. RX respons:

    21 00 00 2f 01 29 a3 1a 3d 33 e3 8a 3a 00 00 00 00 23 0d 32 1c 00 50 50

Send second messange stop applied torque. RX respons:

    0

----

# improving performance


    sudo gedit /etc/modprobe.d/pcan.conf

.

    # pcan - automatic made entry, begin --------
    # if required add options and remove comment
    # options pcan type=isa,sp
    options pcan fdirqtl=1
    options pcan fdirqcl=1
    options pcan fdusemsi=1
    install pcan modprobe --ignore-install pcan
    # pcan - automatic made entry, end ----------

Save the file and run the follwing. (which unloads and then reloads the drive)

    sudo rmmod pcan
    sudo modprobe pcan

Set a device-number for specific channel
Check current device number (replace [pcanpcifd0] with results from ls -l /dev/pcan*)

    pcan-settings -f=/dev/pcanpcifd0 –d

Apply a device id to a channel (in this example id:10)

    pcan-settings -f=/dev/pcanpcifd0 –d 10

Confirm and assign ID:s to each channel. Then we can use #define DEVICE "/dev/pcan-pcie_fd/devid=X to call the joint

moteus_tool and tview configuration over python-can

- ## Install Pcan basic api

Download : https://www.peak-system.com/quick/BasicLinux

    To build PCAN-Basic library:
    > cd libpcanbasic/pcanbasic
    > make clean
    > make 


    To install PCAN-Basic library (inside pcanbasic directory):
    > sudo make install
    (or as root "make install")


    To uninstall PCAN-Basic library (inside pcanbasic directory):
    > sudo make uninstall
    (or as root "make uninstall")


reference: https://python-can.readthedocs.io/en/master/interfaces/pcan.html

- Create an ~/can.conf

Add the following to the can.conf

    [default]
    interface = pcan
    fd = True
    f_clock_mhz = 80
    nom_brp = 1
    nom_tseg1 = 50
    nom_tseg2 = 29
    nom_sjw = 10
    data_brp = 1
    data_tseg1 = 8
    data_tseg2 = 7
    data_sjw = 12

open a terminal :

    python3.8 -m moteus_gui.tview --devices=5 --can-iface pcan --can-chan PCAN_PCIBUS1
