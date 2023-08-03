#ifndef EKFNODE_H
#define EKFNODE_H

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <iostream>
#include <vector>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <Eigen/Dense>
#include "diff_drive/Imu_Reader.h"
#include "diff_drive/Odom_Reader.h"

/**
 * @brief Class for EKF based State Estimation
 * Estimates (x,y,yaw) based on odometry (v and omega) based predictions and IMU Accelerometer and Gyroscope yaw measurements
 * 
 */
class EKFNode: public rclcpp::Node
{
    public:
        /**
         * @brief Construct a new EKFNode object
         * 
         * @param odom_topic_name    Topic to be subscribed to in order to read odometry information
         * @param imu_topic_name     Topic to be subscribed to in order to read IMU measurement data
         */
        EKFNode(std::string odom_topic_name, std::string imu_topic_name): Node("EKF_node"){
            // Reentrant callback group ensures callbacks work paralelly using multi-threading
            odom_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
            imu_cb_group_= this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
            rclcpp::SubscriptionOptions odom_sub_options;
            odom_sub_options.callback_group = odom_cb_group_;
            rclcpp::SubscriptionOptions imu_sub_options;
            odom_sub_options.callback_group = imu_cb_group_;
            odom_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_name, rclcpp::QoS(rclcpp::KeepLast(10)).best_effort(), std::bind(&EKFNode::odom_callback, this, std::placeholders::_1), odom_sub_options);
            imu_subscription_ = this->create_subscription<sensor_msgs::msg::Imu>(
      imu_topic_name, rclcpp::QoS(rclcpp::KeepLast(10)).best_effort(), std::bind(&EKFNode::imu_callback, this, std::placeholders::_1), imu_sub_options);
            ekf_publisher_ = this->create_publisher<geometry_msgs::msg::Pose>("/ekf_state", 10);

        }
    private:
        /**
         * @brief Callback for Odometry Subscriber (Performs Prediction Update)
         * 
         * @param msg const nav_msgs::msg::Odometry::SharedPtr
         */
        void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
        /**
         * @brief Callback for IMU Subscriber (Performs Measurement Update and Publishes EKF Estimated Pose)
         * 
         * @param msg const sensor_msgs::msg::Imu::SharedPtr
         */
        void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg);
        /**
         * @brief Helper method to get the Yaw from Quaternion object
         * 
         * @param quaternion 
         * @return double returns yaw
         */
        double getYawfromQuaternion(const geometry_msgs::msg::Quaternion& quaternion);
        // Shared pointers to callback groups, subscriber and publisher
        rclcpp::CallbackGroup::SharedPtr odom_cb_group_;
        rclcpp::CallbackGroup::SharedPtr imu_cb_group_;
        rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr ekf_publisher_;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
        rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscription_;
        // State
        Eigen::Vector3d state_ = Eigen::Vector3d::Zero();
        // State Covariance Matrix
        Eigen::Matrix3d P_ {{10,0,0},{0,10,0},{0,0,10}};
        
        // Measurement Noise Covariance Matrix
        double R_ = 0.01;
        // Process Noise Covariance Matrix
        Eigen::Matrix3d Q_ {{1,0,0},{0,1,0},{0,0,1}};
        //Initial time
        rclcpp::Time prev = this->get_clock()->now();  
        


};




#endif