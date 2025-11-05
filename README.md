# robot_cpp_learning


project folder:
`ros2_ws/src/robot_learning/`


```
cd ~/ros2_ws
# 安装OpenCV（若未装）：sudo apt install libopencv-dev ros-humble-cv-bridge ros-humble-image-transport
# Pi/Jetson：安装wiringPi/Jetson.GPIO，并CMake时加 -D RASPI=ON
colcon build --symlink-install
source install/setup.bash
```
