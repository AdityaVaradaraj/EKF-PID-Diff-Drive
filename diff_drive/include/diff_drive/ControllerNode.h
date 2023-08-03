#ifndef CONTROLLERNODE_H
#define CONTROLLERNODE_H
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <iostream>
#include <vector>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include "diff_drive/Robot_Motion.h"

/**
 * @brief Class for PID Control of Differential drive robot based on the current estimted state from EKF and reference trajectory. 
 * 
 */
class ControllerNode: public rclcpp::Node
{
    public:
        /**
         * @brief Construct a new Controller Node object
         * 
         * @param ekf_topic_name Topic to subscribe to in order to read estimated state published by EKF node
         * @param Kp_m_ Proportional Gain for error in Magnitude
         * @param Ki_m_ Integral Gain for error in Magnitude
         * @param Kd_m_ Derivative Gain for error in Magnitude
         * @param Kp_y_ Proportional Gain for error in Yaw (direction)
         * @param Ki_y_ Integral Gain for error in Yaw (direction)
         * @param Kd_y_ Derivative Gain for error in Yaw (direction)
         * @param mode_ Mode, i.e., simulation (Turtlebot3) or reality (actual diff drive robot)
         * @param L_ Wheelbase of robot
         * @param r_ Wheel radius of robot
         */
        ControllerNode(std::string ekf_topic_name, double Kp_m_, double Ki_m_, double Kd_m_, double Kp_y_, double Ki_y_, double Kd_y_, std::string mode_, double L_, double r_): Node("Controller_node"){
            L = L_;
            r = r_;
            Kp_m = Kp_m_;
            Ki_m = Ki_m_;
            Kd_m = Kd_m_;
            Kp_y = Kp_y_;
            Ki_y = Ki_y_;
            Kd_y = Kd_y_;
            mode = mode_;
            ctrl_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
            rclcpp::SubscriptionOptions ctrl_sub_options;
            ctrl_sub_options.callback_group = ctrl_cb_group_;
            ekf_subscription_ = this->create_subscription<geometry_msgs::msg::Pose>(
      ekf_topic_name, rclcpp::QoS(rclcpp::KeepLast(10)).best_effort(), std::bind(&ControllerNode::ctrl_callback, this, std::placeholders::_1), ctrl_sub_options);
            vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        }
    private:
        /**
         * @brief Callback function for EKF topic (Estimated State) subscriber. Implements PID Control and sends velocity commands to robot
         * 
         * @param msg Pose message containing Estimated State data
         */
        void ctrl_callback(const geometry_msgs::msg::Pose::SharedPtr msg);
        /**
         * @brief Get the Yaw From Quaternion object
         * 
         * @param quaternion 
         * @return double representing yaw
         */
        double getYawFromQuaternion(const geometry_msgs::msg::Quaternion& quaternion);
        /**
         * @brief Get the Magnitude (Euclidean Norm) of the point vector given by x and y
         * 
         * @param x 
         * @param y 
         * @return double returns Euclidean norm
         */
        double getMagnitude(double x, double y);
        // Proportional Gain for magnitiude
        double Kp_m = 1e-6;
        // Derivative Gain for magnitude
        double Kd_m = 0;
        // Integral Gain for magnitude
        double Ki_m = 0;
        // Proportional Gain for yaw
        double Kp_y = 1e-5;
        // Derivative Gain for yaw
        double Kd_y = 0;
        // Integral Gain for yaw
        double Ki_y = 0;
        // For integral
        double integral_mag = 0;
        // For integral
        double integral_yaw = 0;
        // For derivative
        double prev_error_mag = 0;
        // For derivative
        double prev_error_yaw = 0;
        // Robot Wheelbase
        double L = 0.287;
        // Robot wheel radius
        double r = 0.066;
        // Mode: simulation or reality
        std::string mode = "simulation";
        // Reentrant Callback group for EKF Subscriber Callback which implements PID Control
        rclcpp::CallbackGroup::SharedPtr ctrl_cb_group_;
        // Publisher to /cmd_vel if simulation. (Not used in reality mode)
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_publisher_;
        // Subscriber to EKF estimated state
        rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr ekf_subscription_;
};

#endif