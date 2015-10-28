close all; clear all; clc;

PC_IP = '192.168.173.1';
BEAGLEBONE_IP = 'http://192.168.173.108';
PLOT_INTERVAL = 500;

% Initialise ROS on remote master
setenv('ROS_MASTER_URI', strcat(BEAGLEBONE_IP, ':11311'))
setenv('ROS_IP', PC_IP)
rosinit

sub = rossubscriber('/hovercraft/imu_readings', rostype.sensor_msgs_Imu);

fig = figure; hold on;
cnt = 0; T = []; X = []; Y = []; Z = [];
while 1
    imu_reading = receive(sub);
    time = rostime('now');

    cnt = cnt + 1;
    T = [T cnt];
    X = [X imu_reading.LinearAcceleration.X];

    figure(fig);
    plot_topic(T, X, 'X acceleration', 'r', 100);
end
