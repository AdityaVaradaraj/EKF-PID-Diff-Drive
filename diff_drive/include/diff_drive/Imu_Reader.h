#ifndef IMU_READER_H
#define IMU_READER_H

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <iostream>
#include <vector>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/msg/imu.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <Eigen/Dense>

/**
 * @brief Class to read data from physical IMU (MPU6050) and publish to a ROS2 topic
 * 
 */
class Imu_Reader: public rclcpp::Node{
    public:
        /**
         * @brief Construct a new Imu_Reader object
         * 
         * @param imu_topic_name name of topic to publish IMU data on
         * @param bus_number Bus Number on which the IMU sensor is connected
         * @param device_address Device Address for the IMU Sensor
         */
        Imu_Reader(std::string imu_topic_name, const int& bus_number, const int& device_address): Node("Imu_Reader_Node"){
            imu_publisher_ = this->create_publisher<sensor_msgs::msg::Imu>(imu_topic_name, 10);
            // Open the I2C bus
            openI2CBus(bus_number);
            
            // Set the I2C device address
            if (ioctl(i2c_bus, I2C_SLAVE, device_address) < 0) {
                close(i2c_bus);
                throw std::runtime_error("Failed to set I2C device address");
            }
        }
        /**
         * @brief Destroy the Imu_Reader object
         * 
         */
        ~Imu_Reader(){
            // Close the I2C Bus in Destructor
            close(i2c_bus);
        }
    private:
        /**
         * @brief Reads IMU Sensor data from MPU6050 and publishes it on the ROS2 topic
         * 
         */
        void read_imu();
        /**
         * @brief Calculate Roll-Pitch-Yaw data from accelerometer data (accelerations in X,Y and Z) and gyroscope data (Angular velocities about X,Y and Z)
         * 
         * @param accel_data 
         * @param gyro_data 
         * @return Eigen::Vector3d Vector containing roll, pitch and yaw
         */
        Eigen::Vector3d getRPYFromAccGyro(const Eigen::Vector3d& accel_data, const Eigen::Vector3d& gyro_data);
        /**
         * @brief Opens I2C bus with given bus number
         * 
         * @param bus_number 
         */
        void openI2CBus(const int& bus_number) {
            std::string device_name = "/dev/i2c-" + std::to_string(bus_number);
            i2c_bus = open(device_name.c_str(), O_RDWR);
            if (i2c_bus < 0) {
                throw std::runtime_error("Failed to open I2C bus: " + device_name);
            }
            
        }
        // Publisher that publishes IMU data
        rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher_;
        // Previous time
        rclcpp::Time prev = this->get_clock()->now();
        // Current time 
        rclcpp::Time now = this->get_clock()->now();
        // Complementary Filter parameter for combining accelerometer and gyroscope data 
        double alpha = 0.98;
        // RPY from gyroscope
        Eigen::Vector3d gyro_rpy = Eigen::Vector3d::Zero();
        // I2C Bus
        int i2c_bus;
};



#endif