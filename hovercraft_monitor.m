close all; clear all; clc;

PC_IP = '192.168.7.1';
BEAGLEBONE_IP = 'http://192.168.7.2';
PLOT_INTERVAL = 500;

% Initialise ROS on remote master
setenv('ROS_MASTER_URI', strcat(BEAGLEBONE_IP, ':11311'))
setenv('ROS_IP', PC_IP)
rosinit

imu_sub = rossubscriber('/hovercraft/imu_readings', rostype.sensor_msgs_Imu, 'BufferSize', 1);
cam_sub = rossubscriber('/hovercraft/usb_cam/image_raw', rostype.sensor_msgs_Image, 'BufferSize', 1);

fig = figure('Visible', 'off', 'Name','Robocape monitoring GUI'); hold on;
cnt = 0; T = [];
AccX = []; AccY = []; AccZ = [];
GyroX = []; GyroY = []; GyroZ = [];
CompX = []; CompY = []; CompZ = [];
while 1
    imu_reading = receive(imu_sub);
    cam_image = receive(cam_sub);
    time = rostime('now');

    cnt = cnt + 1;
    T = [T cnt];
    AccX = [AccX imu_reading.LinearAcceleration.X];
    AccY = [AccY imu_reading.LinearAcceleration.Y];
    AccZ = [AccZ imu_reading.LinearAcceleration.Z];
    GyroX = [GyroX imu_reading.AngularVelocity.X];
    GyroY = [GyroY imu_reading.AngularVelocity.Y];
    GyroZ = [GyroZ imu_reading.AngularVelocity.Z];

    Orientation = quat2eul([imu_reading.Orientation.X, imu_reading.Orientation.Y, imu_reading.Orientation.Z, imu_reading.Orientation.W]);
    CompX = [CompX Orientation(1)];
    CompY = [CompY Orientation(2)];
    CompZ = [CompZ Orientation(3)];

    img = readImage(cam_image);
    monitoring_ui(fig, T, img, ...
                  [AccX; AccY; AccZ], 'Linear acc', ...
                  [GyroX; GyroY; GyroZ], 'Angular vel', ...
                  [CompX; CompY; CompZ], 'Compass', ...
                  20)

    % pause(0.2)
end
