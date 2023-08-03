#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <iostream>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <Eigen/Dense>
#include "diff_drive/EKFNode.h"
#include "diff_drive/ControllerNode.h"
#include "diff_drive/Imu_Reader.h"
#include "diff_drive/Odom_Reader.h"

using namespace std::chrono_literals;


int main(int argc, char * argv[])
{

    rclcpp::init(argc, argv);
    
    // PID Controller Gains - Need to be tuned properly for each trajectory
    double Kp_m = 1e-10;
    double Ki_m = 0.0;
    double Kd_m = 0.0;
    double Kp_y = 10.0;
    double Ki_y = 0.0;
    double Kd_y = 0.0;


    // Robot dimensions
    // Robot Wheelbase
    double L = 0.287;
    // Robot wheel radius
    double r = 0.066;
    // Mode can be either "simulation" or "reality"
    std::string mode = "simulation";
    

    
    // Instantiate the nodes for EKF and PID Control
    std::shared_ptr<EKFNode> ekf_node = std::make_shared<EKFNode>("/odom", "/imu");
    std::shared_ptr<ControllerNode> control_node = std::make_shared<ControllerNode>("/ekf_state", Kp_m, Ki_m, Kd_m, Kp_y, Ki_y, Kd_y, mode, L, r);

    

    // Initialize Executor
    auto executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
    // Add nodes to executor
    executor->add_node(ekf_node);
    executor->add_node(control_node);

    if(mode == "reality"){
        // Nodes for reading from MPU6050 IMU and AS5600 encoder
        int imu_bus_number = 1;
        int imu_device_address = 0x68;
        std::shared_ptr<Imu_Reader> imu_node = std::make_shared<Imu_Reader>("/imu", imu_bus_number, imu_device_address);
        int odom_bus_number_1 = 2;
        int odom_bus_number_2 = 3;
        int odom_device_address = 0x36;
        std::shared_ptr<Odom_Reader> odom_node = std::make_shared<Odom_Reader>("/odom", odom_bus_number_1, odom_bus_number_2, odom_device_address, L, r);
        executor->add_node(imu_node);
        executor->add_node(odom_node);
    }

    // Spin the multithreaded executor
    executor->spin();

    // Shut down ROS2 at the end
    rclcpp::shutdown();
    return 0;
}