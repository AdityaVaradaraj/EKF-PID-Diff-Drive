# EKF-PID-Diff-Drive
Apply EKF State Estimation on IMU data and Odometry information coming from Robot (either turtlebot3 or real robot), use the estimated state to apply PID control for reference trajectory tracking, send velocity commands to the robot.

## Gazebo Simulation

https://drive.google.com/file/d/1znT5TkfQugAseCmxBbOOp1UmyG3KvECt/view?usp=drive_link

## Robot configuration assumed:

1) In simulation mode, Turtlebot3 Waffle
2) In reality mode, Robot with MPU6050 IMU, AS5600 Wheel Encoders, PCA9685 Motor Driver, 37D Metal Gear 6.25:1 12V Motors.  

## Requirements:

C++17

ROS2 Humble

Eigen3 (3.3 or above) (libeigen3-dev)

tf2

tf2_ros

libi2c-dev

## Instructions:

### Simulation:

1) Install required packages
2) Create a ROS2 package named diff_drive with ament_cmake (```ros2 pkg create --build-type ament_cmake diff_drive```)
3) Unzip given package and copy contents of src, include, CMakeLists.txt and package.xml into corresponding files/directories of the created ROS2 package.
4) Set PID Gains, Robot dimensions and mode as "simulation" in main.cpp. Set trajectory (```x_des,y_des, dydt_des, dxdt_des```) in ControllerNode.cpp. 
5) Save and ```colcon build```
6) ```source install/setup.bash```
7) In another terminal, ```ros2 launch turtlebot3_gazebo empty_world.launch``` (Launch the Gazebo world)
8) In the first terminal, ```ros2 run diff_drive diff_drive``` 
9) If doesn't follow trajectory as expected, interrupt the programs in both the terminals. Tune the PID Gains.
10) Repeat 5,6,7,8,9 till gains are well-tuned and expected behavior is seen.


### Reality:

1) Install required packages
2) Create a ROS2 package named diff_drive with ament_cmake (```ros2 pkg create --build-type ament_cmake diff_drive```)
3) Unzip given package and copy contents of src, include, CMakeLists.txt and package.xml into corresponding files/directories of the created ROS2 package.
4) Connect the IMU, encoders and motor driver to laptop using I2C. Make sure motors are connected properly to motor drivers.
5) Edit Bus numbers, device addresses, register numbers, mode (set as "reality") accordingly in the code (main.cpp and ControllerNode.cpp). Set trajectory (```x_des,y_des, dydt_des, dxdt_des```) in ControllerNode.cpp. 
6) Save, ```colcon build```, and ```source install/setup.bash```
7) ```ros2 run diff_drive diff_drive```
8) If doesn't follow trajectory as expected, tune the PID Gains.
9) Repeat 6,7,8 till gains are well-tuned and expected behavior is seen.


