# Servo Speed Control & Arduino ROS driver
This package contains a ROS driver for microcontrollers and mulitple servo-motor speed control. The microcontroller script publishes the analog feedback (mV) and the remapper class handles all necessary mappings.

# Installation

## Tested on:
1) Ubuntu 20.04 with ROS Noetic
2) Teensy 4.0 

## Install ROS serial for Noetic
```
$ sudo apt-get install ros-noetic-rosserial
$ sudo apt-get install ros-noetic-rosserial-arduino
```
### Update System
```
$ sudo apt-get update
```

##   For *teensy* download the rules
Run:
```
$ wget https://www.pjrc.com/teensy/00-teensy.rules 
$ sudo cp 00-teensy.rules /etc/udev/rules.d/ 
```


## Install Arduino IDE + Teensyrduino
Go to the folder scripts/shell_scripts that contains **install_ros_arduino.sh**

```
$ chmod +x install_ros_arduino.sh
$ ./install_ros_arduino.sh
```

## Install rosserial to arduino IDE
1. Open arduino IDE
2. Tools -> Manage Libraries
3. search rosserial and install

# Test rosserial & teensyrduino installation

Open the "Blink" example from rosserial and upload the sketch example. Then:
```
$ roscore
$ rosrun rosserial_python serial_node.py /dev/ttyACM0
$ rostopic list 
$ rostopic pub /toggle_led std_msgs/Empty "{}"
```

## Make a workspace and get a copy of the repo
```
$ mkdir -p arduino_ws/src
$ cd ~/<your_workspace>/src
$ git clone https://github.com/MichaelMarav/ardui_ros.git
$ cd ../ && catkin_make
```

## Make Arduino libraries (adding the custom rosmsgs)
Source your ROS workspace:
```
$ source <workspace_name>/devel/setup.bash
```

Go to *scripts/shell_scripts/*
and run:
```
./build_rosmsgs.sh
```
If you don't want to do it automatically follow the tutorial below:

```
$ cd ~/Arduino/libraries
$ rosrun rosserial_client make_libraries .
```
This should create a "ros_lib/" folder

Now move to:
```
$ cd ~/Arduino/libraries/Rosserial_Arduino_Library/src
$ cp -r ~/Arduino/libraries/ros_lib/ardui_ros/ ./
$ rm -rf ~/Arduino/libraries/ros_lib/
```


(Don't think this is necessary but in case of any problems run this too and repeat the process):
```
$ rosrun rosserial_client make_library.py ~/Arduino/libraries <package_with_msgs>
```
----------------------

