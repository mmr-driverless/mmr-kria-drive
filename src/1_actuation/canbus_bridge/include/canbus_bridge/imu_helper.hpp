/* IMU Orientation Setter */
inline void setImuOrientation(sensor_msgs::msg::Imu& imu_data, const float qx, const float qy,const float qz, const float qw)
{
    imu_data.orientation.x = qx;
    imu_data.orientation.y = qy;
    imu_data.orientation.z = qz;
    imu_data.orientation.w = qw;
}

/* IMU Angular Velocity Setter */
inline void setImuAngularVelocity(sensor_msgs::msg::Imu& imu_data, const float gyro_x, const float gyro_y,const float gyro_z)
{
    imu_data.angular_velocity.x = gyro_x;
    imu_data.angular_velocity.y = gyro_y;
    imu_data.angular_velocity.z = gyro_z;
}


/* IMU Linear Acceleration Setter */
inline void setImuLinearAcceleration(sensor_msgs::msg::Imu& imu_data, const float acc_x, const float acc_y,const float acc_z)
{
    imu_data.linear_acceleration.x = acc_x;
    imu_data.linear_acceleration.y = acc_y;
    imu_data.linear_acceleration.z = acc_z;
}
