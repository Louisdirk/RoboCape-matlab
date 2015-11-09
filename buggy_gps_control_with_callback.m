%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Author:   Mathieu Bresciani
% Date:     28.oct.2015
% Description:
%   This script joins an existing ROS network, subscribes to GNSS data via
%   a callback function and publishes PWM commands.
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

% Subscribe to IMU data
global imu_data
global imu_new_data
imu_sub = rossubscriber('/car/imu_readings', rostype.sensor_msgs_Imu, @imuCallback);
mag_sub = rossubscriber('/car/mag_readings', rostype.sensor_msgs_MagneticField, @magCallback);

acc = zeros(3,1);
gyro = zeros(3,1);
mag = zeros(3,1);

% Subscribe to GNSS data
global car_lat
global car_lon
global gps_new_data

gnss_sub = rossubscriber('/car/fix', rostype.sensor_msgs_NavSatFix, @gpsCallback);

% Initialize origin
msg = receive(gnss_sub);
lat0 = msg.Latitude;
lon0 = msg.Longitude;

% Define control loop period
loop_period = 0.050;

figure(1);
xPos = 0;
yPos = 0;
h = plot(xPos,xPos, 'o');
axis([-15 15 -15 15]);
xlabel('Meters towards East');
ylabel('Meters towards North');

% Stops the car when any key is pressed
global stop_car
stop_car = 0;
gcf
set(gcf, 'KeyPressFcn', @myKeyPressFcn);

t_run = 0;
t = 0;
dt = 0;
computation_time = 0;

while ~stop_car
    tic;
    
    if imu_new_data == 1
       acc = [acc imu_data.accelerometer'];
       gyro = [gyro imu_data.gyroscope'];
       mag = [mag imu_data.magnetometer'];
    end
    % Check if new GNSS data is available
    if gps_new_data == 1
        [x, y] = latlon2carthesian(lat0, lon0, car_lat, car_lon);
        gps_new_data = 0;
        xPos = [xPos x];
        yPos = [yPos y];
        set(h,'XData',xPos,'YData',yPos);
        d = sqrt(x*x+y*y);
        fprintf('\t Longitude:%3.6f \t Latitude:%3.6f \t Altitude:%3.6f\n', msg.Longitude, msg.Latitude, msg.Altitude);
        fprintf('Coordinates:  \n\t x: %2.2f (m)\t y: %2.2f (m)\t d: %2.2f (m)\n', x, y, d);
    end
    
    % Update car speed and steering angle
    steeringAngle = -0.0;  % Between -0.54 (left) and 0.54 (right) in radians
    carSpeed = 0;       % In m/s
    
    if carSpeed > 3
        carSpeed = 3;
    end
    
    steeringValue = -0.962*steeringAngle.^2 + 1.434*steeringAngle;
    engineValue = 0.84*carSpeed+2.543;                            % Up to 10
    
    if carSpeed < 0.5
        engineValue = 0;
    end
    % Failsafe
    if engineValue > 6
        engineValue = 6;
    end
    engine_msg.Data = engineValue-12.5;
    steering_msg.Data = steeringValue-0.435;

    send(engine_pub, engine_msg)
    send(steering_pub, steering_msg)
    
    % fprintf('\n Loop \n');
    % Get time elapsed in current loop
    loop_t = toc;
    computation_time = [computation_time loop_t];
    if loop_t > loop_period
        fprintf('/!\\ Warning : RealTime issue \n\t Last loop period: %.4f(s) instead of %1.4f(s)\n', loop_t, loop_period);
    end
    
    % Wait until total loop time is elapsed
    pause(loop_period - loop_t - 0.01);
    dt = [dt toc];
    t_run = t_run + toc;
    t = [t t_run];
    % toc
end

% Stop
engine_msg.Data = 0-12.5;
steering_msg.Data = 0-0.435;


send(engine_pub, engine_msg)
send(steering_pub, steering_msg)

% pause(1);
% 
% rosshutdown;