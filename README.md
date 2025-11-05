# robot_cpp_learning


project folder:
`ros2_ws/src/robot_learning/`




## 项目文件结构


- **OpenCV**：在相机节点中使用真实图像捕获和发布（`sensor_msgs/Image`），并新增一个图像处理节点用于简单障碍物检测（基于边缘检测）。
- **RT / QoS**：在所有ROS2节点中添加QoS设置（使用`Reliable`可靠性、`KeepLast`历史、`SystemDefault`寿命，并设置`deadline`为实时性调优）。假设使用Cyclone DDS（ROS2默认），这可提升DDS通信的实时性和可靠性。
- **PID控制**：新增PID控制器类，用于电机节点，实现平滑速度控制（基于目标速度的误差积分微分调节）。
- **实车发布版**：电机类扩展为硬件驱动，支持Raspberry Pi（使用`wiringPi`库模拟GPIO控制电机）或Jetson（使用标准GPIO）。CMake中添加条件编译（`if(RASPI)`或`if(JETSON)`）。**注意**：实际部署需在目标硬件上安装对应库（如`wiringPi` for Pi），这里提供代码框架；模拟模式下仍用虚拟控制。





### 任务代码框架

+ **1) 电机类：构造/析构 + 方法（OOP + RAII） + 硬件驱动 + PID集成**

**新增：`pid_controller.hpp`**（PID类，实现平滑控制）


+ **2) Sensor 接口 + 派生类（多态）**

+ **3) Camera 派生类（继承 + 资源管理）**

+ **4) 多线程模拟传感器采集** 




+ **5) ROS2 电机控制节点（集成PID + QoS + 硬件）**


+ **6) ROS2 相机发布节点（集成OpenCV真实捕获 + QoS）**


+ **7) 新增：ROS2 图像处理节点（OpenCV障碍检测 + QoS）**


+ **新增：`utils.hpp`**（QoS调优辅助）


---

## 运行方法

### **准备（Ubuntu + ROS2 Humble/Foxy）**
```bash
cd ~/ros2_ws
# 安装OpenCV（若未装）：sudo apt install libopencv-dev ros-humble-cv-bridge ros-humble-image-transport
# Pi/Jetson：安装wiringPi/Jetson.GPIO，并CMake时加 -D RASPI=ON
colcon build --symlink-install
source install/setup.bash
```

### **测试基础节点**
```bash
ros2 run robot_cpp_learning ros2_motor_node  # 启动电机（PID平滑）
ros2 topic pub /motor_cmd std_msgs/msg/Int32 "{data: 80}"  # 测试命令
```

### **测试相机 + 处理（自动避障）**
```bash
# 终端1：相机节点（需接USB相机）
ros2 run robot_cpp_learning ros2_camera_node

# 终端2：图像处理（检测障碍，发布/motor_cmd）
ros2 run robot_cpp_learning ros2_image_process_node

# 终端3：电机响应（观察平滑控制）
ros2 run robot_cpp_learning ros2_motor_node
ros2 topic echo /motor_cmd  # 观察避障命令
```

### **实车部署提示**
- **Raspberry Pi**：`colcon build -D RASPI=ON`，接L298N电机驱动到GPIO 0（PWM）。
- **Jetson**：`colcon build -D JETSON=ON`，类似GPIO配置。
- **全链路测试**：相机捕获 → 处理检测边缘（模拟障碍） → PID电机平滑转向/停止。QoS确保实时（监控`ros2 topic hz /camera/image_raw`）。
- **调试**：用`ros2 topic echo`检查延迟；PID参数调优Kp/Ki/Kd以适应实车惯性。

此架构实现自动避障小车核心：图像→检测→平滑控制→硬件驱动。需进一步优化检测算法（如YOLO for 真障碍）。有问题随时问！
