close all; clear all; clc;

PC_IP = '192.168.7.1';
BEAGLEBONE_IP = 'http://192.168.7.2';

% Initialise ROS on remote master
setenv('ROS_MASTER_URI', strcat(BEAGLEBONE_IP, ':11311'))
setenv('ROS_IP', PC_IP)
rosinit

sub = rossubscriber('/hovercraft/fix', rostype.sensor_msgs_NavSatFix);

while 1
    msg = receive(sub);
    time_now = rostime('now');

    time = mod(time_now.Sec, 86400);
    time_hour = floor(time / 3600);
    time_min = floor(mod(time,3600) / 60);
    time_sec = mod(time, 60);

    fprintf('GPS fix received at %2.0f:%2.0f:%2.0f\n', time_hour, time_min, time_sec);
    fprintf('\t Longitude:%3.6f \t Latitude:%3.6f \t Altitude:%3.6f\n', msg.Longitude, msg.Latitude, msg.Altitude);
end
