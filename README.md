# Servo Speed Control & Arduino ROS driver
This package contains a ROS-based firmware for microcontrollers and mulitple servo-motor open-loop speed control. The microcontroller script publishes the analog feedback (mV), subscribes to the goal angle topic and writes the PWM pulse to the servos. In the following video the goal is for each servo to achieve the same position but with different speeds.


https://user-images.githubusercontent.com/59025730/208120092-7d8511d5-25ff-4ddf-b694-06f7a18ccb49.mp4


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

## Install rosserial using the arduino IDE
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
If the state of the led changes, that means that you have successfully installed the rosserial package.
## Make a workspace and get a copy of the repo
```
$ mkdir -p arduino_ws/src
$ cd ~/<your_workspace>/src
$ git clone https://github.com/MichaelMarav/ardui_ros.git
$ cd ../ && catkin_make
```

## Build the custom rosmsgs
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


In case any problems occurs, run this too and repeat the process:
```
$ rosrun rosserial_client make_library.py ~/Arduino/libraries <package_with_msgs>
```
----------------------

## Usage
To initialize rosserial and the ardui-ros framework, run:

```
$ roslaunch ardui_driver ardui_driver.launch
```
This will initialize the servos at the angles specified at config/initial_state.yaml with the desired velocity.

You should see the /servo/JointStates topic with all the angles of the servos.

In order to command the servos to go anywhere you want with a specific velocity, modify servo_command.py and then run it:

```
$ rosrun ardui_driver servo_command.py
```

