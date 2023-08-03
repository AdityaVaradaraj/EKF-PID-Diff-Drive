#include "diff_drive/Imu_Reader.h"


// Define the register addresses for accelerometer and gyroscope data
constexpr uint8_t MPU6050_ACCEL_XOUT_H = 0x3B;
constexpr uint8_t MPU6050_ACCEL_XOUT_L = 0x3C;
constexpr uint8_t MPU6050_ACCEL_YOUT_H = 0x3D;
constexpr uint8_t MPU6050_ACCEL_YOUT_L = 0x3E;
constexpr uint8_t MPU6050_ACCEL_ZOUT_H = 0x3F;
constexpr uint8_t MPU6050_ACCEL_ZOUT_L = 0x40;

constexpr uint8_t MPU6050_GYRO_XOUT_H = 0x43;
constexpr uint8_t MPU6050_GYRO_XOUT_L = 0x44;
constexpr uint8_t MPU6050_GYRO_YOUT_H = 0x45;
constexpr uint8_t MPU6050_GYRO_YOUT_L = 0x46;
constexpr uint8_t MPU6050_GYRO_ZOUT_H = 0x47;
constexpr uint8_t MPU6050_GYRO_ZOUT_L = 0x48;

/**
 * @brief Function to read a 16-bit signed integer from two consecutive registers
 * 
 * @param i2cBus 
 * @param msbRegAddr 
 * @param lsbRegAddr 
 * @return int16_t 
 */
int16_t readTwoRegisters(int i2cBus, uint8_t msbRegAddr, uint8_t lsbRegAddr) {
    uint8_t msb, lsb;
    if (read(i2cBus, &msb, sizeof(msbRegAddr)) != sizeof(msbRegAddr) ||
        read(i2cBus, &lsb, sizeof(lsbRegAddr)) != sizeof(lsbRegAddr)) {
        throw std::runtime_error("Failed to read from MPU-6050.");
    }
    return static_cast<int16_t>((msb << 8) | lsb);
}


Eigen::Vector3d Imu_Reader::getRPYFromAccGyro(const Eigen::Vector3d& accel_data, const Eigen::Vector3d& gyro_data){
    now = this->get_clock()->now();
    double delta = now.seconds() - prev.seconds();
    // Calculate Gyro RPY using angular velocity data from gyroscope
    gyro_rpy = gyro_rpy + gyro_data*delta;
    // Calculate Accelerometer RPY from Acceleration data in X,Y and Z
    Eigen::Vector3d acc_rpy;
    acc_rpy(0) = std::atan2(accel_data(1), accel_data(2)); // Roll
    acc_rpy(1) = std::atan2(-accel_data(0), std::sqrt(accel_data(1) * accel_data(1) + accel_data(2) * accel_data(2))); // Pitch
    acc_rpy(2) = 0.0; // Yaw (set to 0, as accelerometer data doesn't provide yaw information)

    // Complementary Filter
    Eigen::Vector3d integrated_rpy = alpha*gyro_rpy + (1-alpha)*acc_rpy;

    return integrated_rpy;
}


void Imu_Reader::read_imu(){
    try{
        int16_t accelX = readTwoRegisters(i2c_bus, MPU6050_ACCEL_XOUT_H, MPU6050_ACCEL_XOUT_L);
        int16_t accelY = readTwoRegisters(i2c_bus, MPU6050_ACCEL_YOUT_H, MPU6050_ACCEL_YOUT_L);
        int16_t accelZ = readTwoRegisters(i2c_bus, MPU6050_ACCEL_ZOUT_H, MPU6050_ACCEL_ZOUT_L);
        int16_t gyroX = readTwoRegisters(i2c_bus, MPU6050_GYRO_XOUT_H, MPU6050_GYRO_XOUT_L);
        int16_t gyroY = readTwoRegisters(i2c_bus, MPU6050_GYRO_YOUT_H, MPU6050_GYRO_YOUT_L);
        int16_t gyroZ = readTwoRegisters(i2c_bus, MPU6050_GYRO_ZOUT_H, MPU6050_GYRO_ZOUT_L);

        // Convert raw data to SI units (acceleration in m/s^2, angular velocity in rad/s)
        double accel_scale = 2.0 / 32768.0; // Scale factor for +/-2g range
        double gyro_scale = 250.0 / 32768.0; // Scale factor for +/-250 deg/s range

        Eigen::Vector3d accel_data(
            accelX * accel_scale,
            accelY * accel_scale,
            accelZ * accel_scale
        );

        Eigen::Vector3d gyro_data(
            gyroX * gyro_scale,
            gyroY * gyro_scale,
            gyroZ * gyro_scale
        );
        
        Eigen::Vector3d rpy_angles = getRPYFromAccGyro(accel_data, gyro_data);

        // Calculate the covariance matrix based on accelerometer and gyroscope uncertainties
        // You need to replace these values with the actual uncertainties of your sensors
        // +/-0.5deg is what is mentioned in datasheet of MPU6050
        double accel_covariance = 0.5*3.14/180; // Replace with accelerometer uncertainty
        double gyro_covariance = 0.5*3.14/180;  // Replace with gyroscope uncertainty

        Eigen::Matrix3d orientation_covariance;
        orientation_covariance.setZero();
        orientation_covariance(0, 0) = accel_covariance;
        orientation_covariance(1, 1) = accel_covariance;
        orientation_covariance(2, 2) = gyro_covariance;

        // Convert to quaternion to follow message structure of sensor_msgs/msg/Imu
        tf2::Quaternion q;
        q.setRPY(rpy_angles[0], rpy_angles[1], rpy_angles[2]);


        auto msg = std::make_shared<sensor_msgs::msg::Imu>();
        msg->orientation.x = q.x();
        msg->orientation.y = q.y();
        msg->orientation.z = q.z();
        msg->orientation.w = q.w();

        // Set the orientation covariance matrix
        for (int i = 0; i < 9; ++i) {
            msg->orientation_covariance[i] = orientation_covariance(i / 3, i % 3);
        }


        // Publish the IMU message
        imu_publisher_->publish(*msg);
        
        // Update prev
        prev = now;

    }catch(const std::runtime_error& e){
        std::cerr << "Error while reading from MPU-6050: " << e.what() << std::endl;
    }
}