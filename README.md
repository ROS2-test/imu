## Requirements

Install the serial port library：

    cd serial && mkdir build && cd build
    cmake .. && make
    sudo make install

Install other packages:    

    sudo apt update && sudo apt install libserial-dev  ros-humble-imu-tools

## Usage

Clone the repository in your workspace，and then change directory into the workspace:

    colcon build --packages-select imu  
    source install/setup.bash

## Running the node:

    ros2 run imu imu_node --ros-args --param port_name:=/dev/ttyUSB0

### Launch files

Configurable port, topic name and rate:

    ros2 launch imu imu.launch.py port_name:=/dev/ttyUSB0 topic_name:=/custom_imu/data rate:=250


With rviz :

    ros2 launch imu imu_view.launch.py port_name:=/dev/ttyUSB0



