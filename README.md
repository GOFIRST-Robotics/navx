## navx_node

An ROS node for the NavX IMU from Kauai Labs

Subscribers (inputs)
- update_timer (Timer) : Update loop for reading / querying IMU

Publishers (outputs)
- imu_pub (sensor_msgs/Imu): imu/data : The published imu data
- euler_pub (geometry_msgs/Point): imu/euler : Euler angles RPY (degrees)

Parameters (settings)
- frequency (double): default=50.0 : The update frequency of the update loop
- euler_enable (bool): default=false : Whether to publish euler angles on euler_pub
- device_path (string): default="/dev/ttyACM0" : The serial port path
- frame_id (string): default="imu_link" : IMU message frame ID
- covar_samples (int): default=100 : Number of samples to store to calculate covariance