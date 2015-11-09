function magCallback(src,msg)

    global imu_data
%     global imu_new_data

    imu_data.magnetometer(1) = msg.MagneticField_.X;
    imu_data.magnetometer(2) = msg.MagneticField_.Y;
    imu_data.magnetometer(3) = msg.MagneticField_.Z;
%     imu_new_data = 1; % Synchronized with imu callback
end

