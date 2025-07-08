### before using rplidar 
sudo chmod 666 /dev/ttyUSB0

### boardcast tf
ros2 launch robot_bringup bringup.launch.py

### find error information
ip -details -statistics link show can0

### enable can0
sudo ip link set can0 up type can bitrate 1000000

### send static cmd_vel
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2}, angular: {z: 0.5}}"

### use fewer core
colcon build --parallel-workers 2

### colcon build select pkg
colcon build --packages-select my_package

	
