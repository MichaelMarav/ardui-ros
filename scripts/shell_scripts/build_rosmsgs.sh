#! /bin/sh

rm -rf ~/Arduino/libraries/Rosserial_Arduino_Library/src/ardui_msgs/
cd ~/Arduino/libraries
rosrun rosserial_client make_libraries .
echo "*** Make Libraries ***"
cd ~/Arduino/libraries/Rosserial_Arduino_Library/src
cp -r ~/Arduino/libraries/ros_lib/ardui_msgs/ ./
rm -rf ~/Arduino/libraries/ros_lib/
echo "*** All done,  Go to: -- Arduino/libraries/Rosserial_Arduino_Library/src/ardui_msgs -- and check if the messages exist"
