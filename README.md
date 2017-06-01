# pyKobuki
I wrote this library for the Nvidia Jetson TK1 interfaced with Yuijin Robot Kobuki and an Asus Xtion Pro RGBD camera. The Asus Xtion camera requires openCV and OpenNI2 to run.

Here is what you need to get this library to run:

1. Install the latest Jetpack for the Jetson TK1: https://developer.nvidia.com/embedded/downloads 
2. Since the default firmware for the Jetson comes pretty bare in terms of drivers, install the grinch kernel to get drivers for the Asus Xtion Pro, Wifi module (I use an Atheros AR9285), FTDI FT232RL (serial communication with the Kobuki) : https://devtalk.nvidia.com/default/topic/906018/-customkernel-the-grinch-21-3-4-for-jetson-tk1-developed/ 
3. Install OpenCV. Since the Jetpack provides OpenCV, you only need to install the python bindings :https://devtalk.nvidia.com/default/topic/740390/tegra-jetson-gpu-accleration-in-opencv-/

try: 
sudo apt-get install libopencv4tegra-python

4. Install openNI2: http://myzharbot.robot-home.it/blog/software/configuration-nvidia-jetson-tk1/asus-xtion-pro-live-openni2-compilation-install-instructions/

do go through the comments as there are a few corrections to be made while editing the makefile.

5. Install python bindings for OpenNI2: sudo pip install primesense

6. Try examples here to get familiar with OpenNI2 and openCV to access the RGBD streams: https://github.com/elmonkey/Python_OpenNI2

make sure to modify the path to OpenNI redistribution (path to openNI2.so). This can be found after installing OpenNI2 (step 4)
default path should be: 
dist = '/usr/lib/'

# Functions
constructor:
Kobuki(dev_path)
input:
1. dev_path = path to the FTDI USB-Serial device

This function initializes the Asus Xtion Pro and the Kobuki robot.

send(commands)
input: 
1. commands: list of bytes to be sent to the Kobuki

This function sends commands to the Kobukid
