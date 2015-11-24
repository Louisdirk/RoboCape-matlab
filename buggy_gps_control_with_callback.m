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

% figure;
% while 1
%     msg_mag = receive(mag_sub);
%     phi = atan2(...
%         msg_mag.MagneticField_.X,...
%         msg_mag.MagneticField_.Y...
%         );
%     quiver(zeros(1,length(phi)),zeros(1,length(phi)),cos(phi),sin(phi))
%     axis([-1 1 -1 1]);
% end

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
loop_period = 0.100;

figure;
xPos = 0;
yPos = 0;
phi = 0;
h = plot(xPos,yPos, 'o');
hold all
hq = quiver(xPos, yPos, 0, 0);
axis([-15 15 -15 15]);
xlabel('Meters towards East');
ylabel('Meters towards North');
pause(0.5);
% Stops the car when any key is pressed
global stop_car
stop_car = 0;
gcf;
set(gcf, 'KeyPressFcn', @myKeyPressFcn);

t_run = 0;
t = 0;
dt = 0;
computation_time = 0;
t_imu = 0;
t_gps = 0;

while ~stop_car
    tic;
    drawnow
    if imu_new_data == 1
        imu_new_data = 0;
        acc = [acc imu_data.accelerometer'];
        gyro = [gyro imu_data.gyroscope'];
        mag = [mag imu_data.magnetometer'];
        t_imu = [t_imu t];
    end
    % Check if new GNSS data is available
    if gps_new_data == 1
        [x, y] = latlon2carthesian(lat0, lon0, car_lat, car_lon);
        gps_new_data = 0;
        xPos = [xPos x];
        yPos = [yPos y];
        set(h,'XData',xPos,'YData',yPos);
        phi = [phi atan2(mag(1,end), mag(2,end))];
        set(hq, 'XData',xPos,'YData',yPos, 'UData', 0.1*cos(phi), 'VData', 0.1*sin(phi));
        d = sqrt(x*x+y*y);
        fprintf('\t Longitude:%3.6f \t Latitude:%3.6f \t Altitude:%3.6f\n', msg.Longitude, msg.Latitude, msg.Altitude);
        fprintf('Coordinates:  \n\t x: %2.2f (m)\t y: %2.2f (m)\t d: %2.2f (m)\n', x, y, d);
        t_gps = [t_gps t];
    end
    
    % Update car speed and steering angle
    steeringAngle = 0.1;   % Between -0.54 (left) and 0.54 (right) in radians
    carSpeed = 1;           % In m/s
    
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
    while toc < loop_period
        %
    end
    %     pause(loop_period - loop_t);
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

phi = atan2(mag(2,:),-mag(1,:));
hold all
figure;
quiver(zeros(1,length(phi)),zeros(1,length(phi)),cos(phi),sin(phi))

% pause(1);
%
% rosshutdown;