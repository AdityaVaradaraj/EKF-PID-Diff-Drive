#include "diff_drive/Robot_Motion.h"

constexpr uint8_t PCA9685_ADDRESS = 0x40; // The I2C address of the PCA9685
constexpr uint8_t LED0_ON_L = 0x06;
constexpr uint8_t LED0_ON_H = 0x07;
constexpr uint8_t LED0_OFF_L = 0x08;
constexpr uint8_t LED0_OFF_H = 0x09;



void Robot_Motion::write_speed_for_channel(uint8_t channel, uint16_t speed) {
    // Each channel uses 4 consecutive registers (LEDx_ON_L, LEDx_ON_H, LEDx_OFF_L, and LEDx_OFF_H)
    // Calculate the register addresses for the specific channel
    uint8_t led_on_lsb = LED0_ON_L + (4 * channel);
    uint8_t led_on_msb = LED0_ON_H + (4 * channel);
    uint8_t led_off_lsb = LED0_OFF_L + (4 * channel);
    uint8_t led_off_msb = LED0_OFF_H + (4 * channel);

    uint8_t speed_low_byte = speed & 0xFF;
    uint8_t speed_high_byte = (speed >> 8) & 0xFF;
    
    if (write(i2c_bus, &led_on_lsb, sizeof(led_on_lsb)) != sizeof(led_on_lsb) ||
        write(i2c_bus, &speed_low_byte, sizeof(speed_low_byte)) != sizeof(speed_low_byte) ||
        write(i2c_bus, &led_on_msb, sizeof(led_on_msb)) != sizeof(led_on_msb) ||
        write(i2c_bus, &speed_high_byte, sizeof(speed_high_byte)) != sizeof(speed_high_byte) ||
        write(i2c_bus, &led_off_lsb, sizeof(led_off_lsb)) != sizeof(led_off_lsb) ||
        write(i2c_bus, &speed_low_byte, sizeof(speed_low_byte)) != sizeof(speed_low_byte) ||
        write(i2c_bus, &led_off_msb, sizeof(led_off_msb)) != sizeof(led_off_msb) ||
        write(i2c_bus, &speed_high_byte, sizeof(speed_high_byte)) != sizeof(speed_high_byte)) {
        throw std::runtime_error("Failed to set motor speed for channel " + std::to_string(static_cast<int>(channel)) + ".");
    }
}


void Robot_Motion::move_robot(double v, double omega){
    try{
        // Convert to wheel speeds and send to motor driver
        double omega_l = (v - omega*L/2)*1.0/r;
        double omega_r = (v + omega*L/2)*1.0/r;

        double speed_l = omega_l*60/(2*3.14); // In rpm
        double speed_r = omega_r*60/(2*3.14); // In rpm

        // Duty cycle is 0 to 4095
        // 1600 rpm is max speed for 37D Metal Gear 6.25:1 12V motor
        // Convert to PWM value between 0 to 4095
        speed_l = speed_l/1600*4095; 
        speed_r = speed_r/1600*4095;
        write_speed_for_channel(0,speed_l);
        write_speed_for_channel(1,speed_r);
    }catch(const std::runtime_error& e){
        std::cerr << "Error while writing to 37D Motor using PCA9685 driver: " << e.what() << std::endl;
    }
    



    
}