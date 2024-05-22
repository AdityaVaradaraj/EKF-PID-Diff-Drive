#include "diff_drive/EKFNode.h"

double EKFNode::getYawfromQuaternion(const geometry_msgs::msg::Quaternion &quaternion)
{
    // Convert the quaternion to Euler angles (roll, pitch, yaw)
    tf2::Quaternion tf_quat;
    tf2::convert(quaternion, tf_quat);
    double roll, pitch, yaw;
    tf2::Matrix3x3(tf_quat).getRPY(roll, pitch, yaw);

    return yaw;
}

void EKFNode::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    RCLCPP_INFO(this->get_logger(), "Initial State: [x = '%f', y = '%f', yaw= '%f']", state_[0], state_[1], state_[2]);
    RCLCPP_INFO(this->get_logger(), "Actual State x and y: [x = '%f', y = '%f']", msg->pose.pose.position.x, msg->pose.pose.position.y);
    RCLCPP_INFO(this->get_logger(), "Odometry=[v = '%f', omega = '%f']", msg->twist.twist.linear.x, msg->twist.twist.angular.z);
    // Implement EKF Prediction Update Step
    rclcpp::Time now = this->get_clock()->now();
    double v = msg->twist.twist.linear.x;
    double omega = msg->twist.twist.angular.z;
    double delta = now.seconds() - prev.seconds();
    std::array<double, 36> vel_cov = msg->twist.covariance;

    Eigen::Matrix<double, 6, 6> cov_odom;

    cov_odom << vel_cov[0], vel_cov[1], vel_cov[2], vel_cov[3], vel_cov[4], vel_cov[5],
        vel_cov[6], vel_cov[7], vel_cov[8], vel_cov[9], vel_cov[10], vel_cov[11],
        vel_cov[12], vel_cov[13], vel_cov[14], vel_cov[15], vel_cov[16], vel_cov[17],
        vel_cov[18], vel_cov[19], vel_cov[20], vel_cov[21], vel_cov[22], vel_cov[23],
        vel_cov[24], vel_cov[25], vel_cov[26], vel_cov[27], vel_cov[28], vel_cov[29],
        vel_cov[30], vel_cov[31], vel_cov[32], vel_cov[33], vel_cov[34], vel_cov[35];

    Q_(0, 0) = cov_odom(0, 0);
    Q_(1, 1) = cov_odom(1, 1);
    Q_(2, 2) = cov_odom(5, 5);

    Eigen::Vector3d increment(v * delta * cos(state_[2] + omega * delta), v * delta * sin(state_[2] + omega * delta), omega * delta);
    Eigen::Matrix3d Gk;
    std::cout << "Time step: " << delta << ", Theta new: " << (state_[2] + omega * delta) << std::endl;
    Gk << 1, 0, -1.0 * sin(state_[2] + omega * delta), 0, 1, cos(state_[2] + omega * delta), 0, 0, 1;
    std::cout << "Gk: " << Gk << std::endl;
    // Predict state based on Odometry
    state_ = state_ + increment;
    // Predict state covariance matrix
    P_ = (Gk * P_) * Gk.transpose() + Q_;
    // Update prev
    prev = this->get_clock()->now();
}
double normalizeAngle(double angle)
{
    while (angle > M_PI)
        angle -= 2.0 * M_PI;
    while (angle < -M_PI)
        angle += 2.0 * M_PI;
    return angle;
}
void EKFNode::imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
    RCLCPP_INFO(this->get_logger(), "Initial State: [x = '%f', y = '%f', yaw= '%f']", state_[0], state_[1], state_[2]);
    std::array<double, 9> orientation_covariance = msg->orientation_covariance;
    double epsilon = 1e-3;
    R_ = orientation_covariance[8];
    R_ += epsilon;
    double yaw_measured = getYawfromQuaternion(msg->orientation);
    RCLCPP_INFO(this->get_logger(), "IMU= [yaw = '%f', R = '%f']", yaw_measured, R_);
    Eigen::RowVector3d Ck = Eigen::Vector3d::Zero();
    Ck[2] = 1.0;
    // Implement Measurement Update Step
    // Calculate Kalman Gain (Note that Ck*P_*Ck + R_ is a double since only 1 measurement)
    Eigen::Vector3d K = (P_ * Ck.transpose()) * (1.0 / (Ck.dot(P_ * Ck.transpose()) + R_));
    if (P_ * Ck.transpose() == Eigen::Vector3d::Zero())
    {
        K = Eigen::Vector3d::Zero();
    }
    std::cout << "K: " << K << std::endl;
    double yaw_diff = normalizeAngle(yaw_measured - Ck.dot(state_));

    // Correct State Prediction
    state_ = state_ + K * yaw_diff;
    // Correct State Covariance Matrix Prediction
    P_ = P_ - K * (Ck * P_);
    // Publish States on the /ekf_state topic
    auto msg1 = std::make_shared<nav_msgs::msg::Odometry>();
    msg1->header.stamp = this->get_clock()->now();
    msg1->header.frame_id = "odom";
    msg1->child_frame_id = "base_link";
    msg1->pose.pose.position.x = state_[0];
    msg1->pose.pose.position.y = state_[1];
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, state_[2]);
    msg1->pose.pose.orientation.x = q.x();
    msg1->pose.pose.orientation.y = q.y();
    msg1->pose.pose.orientation.z = q.z();
    msg1->pose.pose.orientation.w = q.w();
    RCLCPP_INFO(this->get_logger(), "Estimated State: [x = '%f', y = '%f', yaw = '%f']", state_[0], state_[1], state_[2]);
    ekf_publisher_->publish(*msg1);
}
