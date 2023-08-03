#ifndef ROBOT_MOTION_H
#define ROBOT_MOTION_H

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <iostream>
#include <vector>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <linux/i2c-dev.h>
// #include <arduino-serial-lib.h>

// PCA9685 Registers
constexpr uint8_t MODE1 = 0x00;
constexpr uint8_t PRESCALE = 0xFE;


/**
 * @brief Class to implement method for sending motion commands to physical robot. Assumption: PCA9685 motor driver with 37D Metal Gear 12V motors used.
 * 
 */
class Robot_Motion{
    public:
        /**
         * @brief Construct a new Robot_Motion object
         * 
         * @param bus_number Bus number for the motor driver
         * @param device_address Device Address for the motor driver
         */
        Robot_Motion(const int& bus_number, const int& device_address, const double& L_, const double& r_){
            // Update Robot dimensions
            L = L_;
            r = r_;
            // Open the I2C bus
            openI2CBus(bus_number);
            
            // Set the I2C device address
            if (ioctl(i2c_bus, I2C_SLAVE, device_address) < 0) {
                close(i2c_bus);
                std::cerr << "Failed to set I2C device address for PCA9685" << std::endl;
            }

            // Configure PCA9685
            uint8_t mode1_data = 0x00; // Normal mode (AI and OE enabled)
            if (write(i2c_bus, &MODE1, sizeof(MODE1)) != sizeof(MODE1) ||
                write(i2c_bus, &mode1_data, sizeof(mode1_data)) != sizeof(mode1_data)) {
                close(i2c_bus);
                std::cerr << "Failed to configure PCA9685." << std::endl;
                
            }

            // Set PWM frequency (50 Hz)
            uint8_t prescale_data = 121; // 25 MHz / (4096 * 50) - 1
            if (write(i2c_bus, &PRESCALE, sizeof(PRESCALE)) != sizeof(PRESCALE) ||
                write(i2c_bus, &prescale_data, sizeof(prescale_data)) != sizeof(prescale_data)) {
                close(i2c_bus);
                std::cerr << "Failed to set PWM frequency." << std::endl;
                
            }

        }
        /**
         * @brief Destroy the Robot_Motion object
         * 
         */
        ~Robot_Motion(){
            // Close the I2C Bus in Destructor
            close(i2c_bus);
        }
        /**
         * @brief Implements Robot motion, i.e., sends wheel speeds as PWM commands to motor driver and hence motors using I2C given v and omega for robot
         * 
         * @param v 
         * @param omega 
         */
        void move_robot(double v, double omega);
    private:
        /**
         * @brief Opens I2C Bus for given bus number 
         * 
         * @param bus_number 
         */
        void openI2CBus(const int& bus_number) {
            std::string device_name = "/dev/i2c-" + std::to_string(bus_number);
            i2c_bus = open(device_name.c_str(), O_RDWR);
            if (i2c_bus < 0) {
                std::cerr << "Failed to open I2C bus for PCA9685: " + device_name << std::endl;
            }
            
        }
        /**
         * @brief Writes speed to particular channel of Motor driver via I2C
         * 
         * @param channel 
         * @param speed 
         */
        void write_speed_for_channel(uint8_t channel, uint16_t speed);
        // Robot Wheelbase
        double L = 0.287;
        // Robot wheel radius
        double r = 0.066;
        // I2C Bus
        int i2c_bus;
};



#endif
