# Description
Generic Pan &amp; Tilt Servos controlled by Arduino Nano via ROSSERIAL

# Install

1) Flash sketch/arduino_ptu.ino onto an Arudino Nano using Arudino IDE

2) Install ptu ROS Driver.

mkdir -p ~/catkin_ws/src

cd ~/catkin_ws/src

git clone https://github.com/ros-drivers/rosserial.git -b noetic-devel

cd ~/catkin_ws/

rosdep install --from-paths src --ignore-src --rosdistro noetic -y -r
catkin_make

# Install Arduino UDEV Rules

sudo cp ~/catkin_ws/src/arduino_ptu_ros/udev/80-arduino.rules /etc/udev/rules.d/

# Troubleshooting Arduino Connection

sudo usermod -a -G dialout $USER

sudo usermod -a -G tty $USER

# Test

rosrun rosserial_python serial_node.py _port:=/dev/arduino _baud:=115200

# Sample Topics

rostopic pub -1 /ptu/cmd_vel geometric_msgs/Twist

rostopic echo /ptu/jointstate
