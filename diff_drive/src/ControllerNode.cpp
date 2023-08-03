#include "diff_drive/ControllerNode.h"


void ControllerNode::ctrl_callback(const geometry_msgs::msg::Pose::SharedPtr msg){
    rclcpp::Time now = this->get_clock()->now();
    double t = now.seconds();
    double x_des = t;
    double y_des = t;
    double dydt_des = 1;
    double dxdt_des = 1;
    double yaw_des = std::atan2(dydt_des, dxdt_des); // Yaw is arctan of trajectory slope dy/dx = (dy/dt) / (dx/dt) 
    double x = msg->position.x;
    double y = msg->position.y;
    double yaw = this->getYawFromQuaternion(msg->orientation);

    double mag_des = getMagnitude(x_des, y_des);
    double mag_curr = getMagnitude(x,y);
    
    double error_mag = mag_des - mag_curr;
    double error_yaw = std::atan2(std::sin(yaw_des - yaw), std::cos(yaw_des - yaw));
    
    double p_term_mag =  Kp_m*(error_mag);
    double p_term_yaw = Kp_y*(error_yaw);
    // Integral can be approximated as sum of errors
    integral_mag += Ki_m*error_mag;
    integral_yaw += Ki_y*error_yaw;

    double i_term_mag = integral_mag;
    double i_term_yaw = integral_yaw;
    // Derivative term can be approximated as difference between consecutive errors
    double d_term_mag = Kd_m*(error_mag - prev_error_mag);
    double d_term_yaw = Kd_y*(error_yaw - prev_error_yaw);
    
    double v = p_term_mag + i_term_mag + d_term_mag;
    double omega = p_term_yaw + i_term_yaw + d_term_yaw;

    // Now command the robot using either Robot_Motion class or using '/cmd_vel' depending on whether testing on turtlebot3 in Gazebo or on actual robot
    
    if(mode == "reality"){
        // For Actual Robot
        // I2C address and register to read from (change as needed)
        int bus_number = 4;
        int device_address = 0x40;
        auto robot_mot = std::make_shared<Robot_Motion>(bus_number, device_address, L, r);
        robot_mot->move_robot(v,omega);
    }else if (mode == "simulation"){
        // For Turtlebot3 using /cmd_vel
        auto vel_msg = std::make_shared<geometry_msgs::msg::Twist>();
        vel_msg->linear.x = v;
        vel_msg->angular.z = omega;
        vel_publisher_->publish(*vel_msg);
    }else{
        std::cerr << "Not a valid mode" << std::endl;
    }
    
    
}

double ControllerNode::getYawFromQuaternion(const geometry_msgs::msg::Quaternion& quaternion){
    // Convert the quaternion to Euler angles (roll, pitch, yaw)
    tf2::Quaternion tf_quat;
    tf2::convert(quaternion, tf_quat);
    double roll, pitch, yaw;
    tf2::Matrix3x3(tf_quat).getRPY(roll, pitch, yaw);

    return yaw;
}

double ControllerNode::getMagnitude(double x, double y){
    return std::sqrt(x*x + y*y);
}