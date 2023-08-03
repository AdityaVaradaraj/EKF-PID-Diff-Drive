#include "diff_drive/Odom_Reader.h"


constexpr uint8_t AS5600_ANGLE_H = 0x0E;
constexpr uint8_t AS5600_ANGLE_L = 0x0F;

/**
 * @brief Function to read a 16-bit signed integer from two consecutive registers
 * 
 * @param i2cBus 
 * @param msbRegAddr 
 * @param lsbRegAddr 
 * @return int 
 */
int readTwoRegisters(int i2cBus, uint8_t msbRegAddr, uint8_t lsbRegAddr) {
    uint8_t msb, lsb;
    if (read(i2cBus, &msb, sizeof(msbRegAddr)) != sizeof(msbRegAddr) ||
        read(i2cBus, &lsb, sizeof(lsbRegAddr)) != sizeof(lsbRegAddr)) {
        throw std::runtime_error("Failed to read data from AS5600.");
    }
    return static_cast<int>((msb << 8) | lsb);
}


void Odom_Reader::read_odom(){
    try{
        now = this->get_clock()->now();
        double delta = now.seconds() - prev.seconds();
        int raw_angle_l = readTwoRegisters(i2c_bus_1, AS5600_ANGLE_H, AS5600_ANGLE_L);
        int raw_angle_r = readTwoRegisters(i2c_bus_2, AS5600_ANGLE_H, AS5600_ANGLE_L);

        // Convert raw data from ANGLE registers to radians
        double angle_l = raw_angle_l*2*3.14/4096;
        double angle_r = raw_angle_r*2*3.14/4096;
        double omega_l = 0;
        double omega_r = 0;
        if(delta > 0){
            omega_l = angle_l/delta;
            omega_r = angle_r/delta;
        }

        // Calculate linear and angular velocities of robot using wheel speeds 
        double v = r*(omega_r + omega_l)/2;
        double omega = r*(omega_r - omega_l)/L;

        // Publish message
        auto msg = std::make_shared<nav_msgs::msg::Odometry>();
        msg->twist.twist.linear.x = v;
        msg->twist.twist.angular.z = omega;
        Odom_publisher_->publish(*msg); 
        
        // Update prev
        prev = now;
    }catch(const std::runtime_error& e){
        std::cerr << "Error while reading from AS5600 encoder: " << e.what() << std::endl;
    }
}