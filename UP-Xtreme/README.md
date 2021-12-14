# ADD KERNEL RT TO UP EXTREME I7 FOR UBUNTU 18.04 AND 20.04

----

## install last official up-xtreme kernel (https://github.com/up-board/up-community/wiki/Ubuntu_20.04)

Install Ubuntu kernel 5.4.0 from PPA on Ubuntu 20.04
This is the latest Kernel available for the UP Series. It has been validated on UP Board, UP Core, UP Squared, UP Core Plus and UP Xtreme.

After the reboot you need to add our repository:

	sudo add-apt-repository ppa:aaeon-cm/5.4-upboard

Update the repository list

	sudo apt update

Remove all the generic installed kernel

	sudo apt-get autoremove --purge 'linux-.*generic'

Install our kernel (18.04 and 20.04 share the same 5.4 kernel):

	sudo apt-get install linux-generic-hwe-18.04-5.4-upboard

Install the updates:

	sudo apt dist-upgrade -y

	sudo update-grub

Reboot

	sudo reboot


After the reboot, you can verify that the kernel is indeed installed by typing

	 $ uname -a
	 Linux upxtreme-UP-WHL01 5.4.0-1-generic #2~upboard2-Ubuntu SMP Thu Jul 25 13:35:27 UTC 2019 x86_64 x86_64 x86_64 GNU/Linux

----

## Hack up kernel

``
sudo apt install build-essential fakeroot dpkg-dev perl libssl-dev bc gnupg dirmngr libncurses5-dev libelf-dev flex bison

mkdir ~/kernel

cd ~/kernel

git clone https://github.com/AaeonCM/ubuntu-bionic-up.git

cd ubuntu-bionic-up

git checkout hwe-5.4-upboard
``

``make kernelversion`` --Gives information on kernel version-- in my case (5.4.65)--

Find RT patch equal or close to kernel version from make kernelversion output

``
wget https://mirrors.edge.kernel.org/pub/linux/kernel/projects/rt/5.4/older/patch-5.4.66-rt38.patch.xz

unxz -cd patch-5.4.66-rt38.patch.xz | patch -p1
``

Create the kernel .config ### Pay attention to use needed kernel (check /boot content !)

cp ../../../../boot/config-5.4.0-1-generic .config


Configure the kernel
xdg-open .config (text based)

Apply RT OPTIONS on kernel
HIGH_RES_TIMERS=y

CONFIG_PREEMPT_RT=y ###if you don't see the full preempt-RT option, you need to activate expert mode on the gui config file(``make menuconfig``)

CONFIG_HZ_1000=y

CONFIG_HZ=1000

CONFIG_OF=n

IF make fails turn off AUFS
CONFIG_AUFS_FS=n

Optional
disable the CPU idle state and to set the default CPU frequency governor to performance

CONFIG_CPU_IDLE=n

CONFIG_INTEL_IDLE=n

CONFIG_CPU_FREQ_DEFAULT_GOV_PERFORMANCE=y

----

## Build

	sudo make -j4

	sudo make modules_install -j4

	sudo make install -j4

Check new kernel file and update grub boot loader to start Linux with new RT-Kernel

	cd /boot

	ls

You will see new RT kernel added.

Update grub and reboot machine

	sudo update-grub

	sudo reboot ###Board reboots with new RT kernel

Type ``uname -a`` to check actual kernel

----

## Real time test using latency plot under the stress
Note: adjust CPU cores on line 2 and 11 in rt-test.sh

``sudo apt install rt-tests stress gnuplot`` 

``mkdir ~/plot``

``cd ~/plot``

``git clone https://github.com/QiayuanLiao/Ubuntu-RT-UP-Board.git``

``cd ~/Ubuntu-RT-UP-Board``

``cd ~/test``

``sudo sh  ./rt-test.sh``

----

## I you need to delete new RT-kernel

``locate -b -e 5.4.65-rt38+ | xargs -p sudo rm -r``

``sudo update-grub``

``cd /boot/``

``ls``

----

## Install upboard-extras

``
sudo add-apt-repository ppa:ubilinux/up sudo apt update

sudo apt install upboard-extras

cd ~/temp

wget https://github.com/raess1/UPxtreme-RT-kernel/blob/main/permissiongroups.sh

chmod +x permissiongroups.sh

./permissiongroups.sh
``

after that you need to add the user that needs to access the HAT functionality to the corresponding groups:

	``GPIO sudo usermod -a -G gpio ${USER}``

	``LEDs sudo usermod -a -G leds ${USER}``

	``SPI sudo usermod -a -G spi ${USER}``

	``I2C sudo usermod -a -G i2c ${USER}``

	``UART sudo usermod -a -G dialout ${USER}``

to apply the permission changes after issuing the previous command a reboot is needed sudo reboot

Test led hat functionality activation :

#### LED test

The UP Board includes 3 LEDs (yellow, green, red) on the underside of the board (underneath the 4 USB2.0 Type-A sockets), 

which are controlled by the pin control CPLD on the board. As root, you can use the following commands to control the LEDs:

``echo 1 > /sys/class/leds/upboard\:green\:/brightness``

``echo 0 > /sys/class/leds/upboard\:green\:/brightness``


# Some tips :

reboot without "sudo"

	``os.system('echo '+pwd+' | sudo -S poweroff')``
