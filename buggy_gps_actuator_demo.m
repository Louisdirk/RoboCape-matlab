close all; clear all; clc;

PC_IP = '10.42.0.33';
BEAGLEBONE_IP = 'http://10.42.0.14';

% Initialise ROS on remote master
setenv('ROS_MASTER_URI', strcat(BEAGLEBONE_IP, ':11311'))
setenv('ROS_IP', PC_IP)
rosinit

engine_pub = rospublisher('/car/actuator_engine_update',rostype.std_msgs_Float64);
engine_msg = rosmessage(engine_pub);

steering_pub = rospublisher('/car/actuator_steering_update',rostype.std_msgs_Float64);
steering_msg = rosmessage(steering_pub);

sub = rossubscriber('/car/fix', rostype.sensor_msgs_NavSatFix);
msg = receive(sub);
lat0 = msg.Latitude;
lon0 = msg.Longitude;

while 1
    msg = receive(sub);
    time_now = rostime('now');

    time = mod(time_now.Sec, 86400);
    time_hour = floor(time / 3600);
    time_min = floor(mod(time,3600) / 60);
    time_sec = mod(time, 60);

    [x, y] = gps2carthesian(lat0, lon0, msg.Latitude, msg.Longitude);
    d = sqrt(x*x+y*y);
    
    engine_msg.Data = 0;
    steering_msg.Data = 0;

    send(engine_pub, engine_msg)
    send(steering_pub, steering_msg)
    
    fprintf('GPS fix received at %2.0f:%2.0f:%2.0f\n', time_hour, time_min, time_sec);
    fprintf('\t Longitude:%3.6f \t Latitude:%3.6f \t Altitude:%3.6f\n', msg.Longitude, msg.Latitude, msg.Altitude);
    fprintf('Position from origin:  \n\t x: %2.2f (m)\t y: %2.2f (m)\t d: %2.2f (m)\n', x, y, d);
end
