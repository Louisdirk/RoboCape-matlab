%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Author:   Mathieu Bresciani
% Date:     28.oct.2015
% Description:
%   This script joins an existing ROS network, subscribes to GNSS data via
%   a callback function and publish pwm commands.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

close all; 
clear all; 
clc;

PC_IP = '10.42.0.33';
BEAGLEBONE_IP = 'http://10.42.0.14';

% Initialise ROS on remote master
setenv('ROS_MASTER_URI', strcat(BEAGLEBONE_IP, ':11311'))
setenv('ROS_IP', PC_IP)
rosinit

% Setup engine control publisher
engine_pub = rospublisher('/car/actuator_engine_update',rostype.std_msgs_Float64);
engine_msg = rosmessage(engine_pub);

% Setup servo control publisher
steering_pub = rospublisher('/car/actuator_steering_update',rostype.std_msgs_Float64);
steering_msg = rosmessage(steering_pub);

% Subscribe to GNSS data
global car_lat
global car_lon
global gps_new_data

sub = rossubscriber('/car/fix', rostype.sensor_msgs_NavSatFix, @gpsCallback);

% Initialize origin
msg = receive(sub);
lat0 = msg.Latitude;
lon0 = msg.Longitude;

% Define control loop period
loop_period = 0.050; 

while 1
    tic;
    
    % Check if new GNSS data is available
    if gps_new_data == 1
        [x, y] = gps2carthesian(lat0, lon0, car_lat, car_lon);
        gps_new_data = 0;
        d = sqrt(x*x+y*y);
        fprintf('\t Longitude:%3.6f \t Latitude:%3.6f \t Altitude:%3.6f\n', msg.Longitude, msg.Latitude, msg.Altitude);
        fprintf('Coordinates:  \n\t x: %2.2f (m)\t y: %2.2f (m)\t d: %2.2f (m)\n', x, y, d);
    end
    
    % Update car speed and steering angle
    engine_msg.Data = 0;
    steering_msg.Data = 0;

    send(engine_pub, engine_msg)
    send(steering_pub, steering_msg)
    
    % Get time elapsed in current loop
    loop_t = toc;
    if loop_t > loop_period
        fprintf('/!\\ Warning : RealTime issue \n\t Last loop period: %.4f(s) instead of %1.4f(s)\n', loop_t, loop_period);
    end
    
    % Wait until total loop time is elapsed
    pause(loop_period - loop_t);
end
