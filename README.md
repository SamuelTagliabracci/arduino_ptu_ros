# Description
Generic Pan &amp; Tilt Servos controlled by Arduino Nano via ROSSERIAL!

# Install
cd ~/catkin_ws/src
git clone https://github.com/ros-drivers/rosserial.git -b noetic-devel
git clone https://github.com/ros-teleop/teleop_tools.git -b kinetic-devel

sudo usermod -a -G dialout $USER
sudo usermod -a -G tty $USER

# Udev - Arduino Rule
SUBSYSTEMS=="usb", ACTION=="add", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="7523", MODE="0666", SYMLINK+="arduino", GROUP="dialout"

# Test
rosrun rosserial_python serial_node.py _port:=/dev/arduino _baud:=115200

# Test Topics
rostopic pub -1 /ptu/cmd_vel geometric_msgs/Twist
rostopic echo /ptu/jointstate

# Sample joy_teleop.yaml addition

teleop:
  ptu:
    type: topic
    message_type: geometry_msgs/Twist
    topic_name: ptu/cmd_vel
    deadman_buttons: [4]
    axis_mappings:
      -
        axis: 3
        target: angular.z
      -
        axis: 4
        target: angular.y
