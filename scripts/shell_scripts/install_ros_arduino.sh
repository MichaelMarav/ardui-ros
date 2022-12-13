#! /bin/sh
cd ~/
sudo apt-get update                                 # Update System
mkdir install_arduino/ && cd install_arduino/       # Create workspace

# Download
wget https://downloads.arduino.cc/arduino-1.8.15-linux64.tar.xz #  arduino IDE
wget https://www.pjrc.com/teensy/td_154/TeensyduinoInstall.linux64 #  teensyrduino
wget https://www.pjrc.com/teensy/00-teensy.rules # rules for usb
sudo cp 00-teensy.rules /etc/udev/rules.d/ # Copy rules
tar -xf arduino-1.8.15-linux64.tar.xz
chmod 755 TeensyduinoInstall.linux64

./TeensyduinoInstall.linux64 --dir=arduino-1.8.15
cd arduino-1.8.15/hardware/teensy/avr/cores/teensy4
make
rospack profile
cd ~/install_arduino/
rm 00-teensy.rules
rm TeensyduinoInstall.linux64
rm arduino-1.8.15-linux64.tar.xz 
echo "*** All done ***"