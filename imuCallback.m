function imuCallback(src,msg)

    global imu_data
    global imu_new_data

    imu_data.accelerometer(1) = msg.LinearAcceleration.X;
    imu_data.accelerometer(2) = msg.LinearAcceleration.Y;
    imu_data.accelerometer(3) = msg.LinearAcceleration.Z;
    
    imu_data.gyroscope(1) = msg.AngularVelocity.X;
    imu_data.gyroscope(2) = msg.AngularVelocity.Y;
    imu_data.gyroscope(3) = msg.AngularVelocity.Z;
    
    imu_new_data = 1;
end

