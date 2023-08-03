#ifndef ODOM_READER_H
#define ODOM_READER_H

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <iostream>
#include <vector>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>

#include "rclcpp/rclcpp.hpp"
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <nav_msgs/msg/odometry.hpp>

/**
 * @brief Class to read Odometry data from physical encoder (AS5600) using I2C and publish on a ROS2 topic
 * 
 */
class Odom_Reader: public rclcpp::Node{
    public:
        /**
         * @brief Construct a new Odom_Reader object
         * 
         * @param odom_topic_name Topic on which to publish odometry data
         * @param bus_number_1 Bus number for Encoder of left wheel
         * @param bus_number_2 Bus number for Encoder of right wheel
         * @param device_address Device Address for Encoder model used
         * @param L_ Wheelbase of Robot
         * @param r_ Wheel radius of robot
         */
        Odom_Reader(std::string odom_topic_name, const int& bus_number_1, int bus_number_2, const int& device_address, const double& L_, const double& r_): Node("Odom_Reader_Node"){
            L = L_;
            r = r_;
            Odom_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>(odom_topic_name, 10);
            // Open the I2C bus
            openI2CBus(i2c_bus_1, bus_number_1);
            
            // Set the I2C device address
            if (ioctl(i2c_bus_1, I2C_SLAVE, device_address) < 0) {
                close(i2c_bus_1);
                throw std::runtime_error("Failed to set I2C device address");
            }

            // Open the I2C bus
            openI2CBus(i2c_bus_2, bus_number_2);
            
            // Set the I2C device address
            if (ioctl(i2c_bus_2, I2C_SLAVE, device_address) < 0) {
                close(i2c_bus_2);
                throw std::runtime_error("Failed to set I2C device address");
            }
            
        }
        /**
         * @brief Destroy the Odom_Reader object
         * 
         */
        ~Odom_Reader(){
            // Close the I2C Bus in Destructor
            close(i2c_bus_1);
            close(i2c_bus_2);
        }
    private:
        /**
         * @brief Read Odometry data from encoders using I2C and publish v and omega in nav_msgs/msg/Odometry format to a ROS2 topic
         * 
         */
        void read_odom();
        /**
         * @brief Opens I2C bus for a given bus number
         * 
         * @param i2c_bus Attribute to be updated
         * @param bus_number bus number given
         */
        void openI2CBus(int& i2c_bus, const int& bus_number) {
            std::string device_name = "/dev/i2c-" + std::to_string(bus_number);
            i2c_bus = open(device_name.c_str(), O_RDWR);
            if (i2c_bus < 0) {
                throw std::runtime_error("Failed to open I2C bus: " + device_name);
            }
        }
        // Publisher for odometry data
        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr Odom_publisher_;
        // Robot Wheelbase
        double L = 0.287;
        // Robot radius
        double r = 0.066;
        // I2C Bus for left wheel
        int i2c_bus_1;
        // I2C Bus for right wheel
        int i2c_bus_2;
        // Time at previous measurement
        rclcpp::Time prev = this->get_clock()->now();
        // Time of current measurement
        rclcpp::Time now = this->get_clock()->now();

};



#endif