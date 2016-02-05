
classdef RealCar < CtSystem & InitDeinitObject
    
    properties
        lastSendTime = 0;
        lastReadTime = 0;
        
        startingCommand          = [];
        waitAfterStartingCommand = 5;
        
        downsampleSendCommand = 1;
        dt;
        
        %        l = 0.34;
        %        lr = 0.17;
        
        engine_msg
        steering_msg
        steering_pub
        engine_pub
        lat0
        lon0
        yaw0
        gps_sub
        imu_sub
        mag_sub

    end
    
    methods
        %         function hP = plot(obj,p,theta)
        %             clr = 1;
        %             R = @(theta)[cos(theta),-sin(theta);
        %                 sin(theta),cos(theta)];
        %             h = obj.l;
        %             w = h/(2);
        %             k = 0.5;
        %             XY=[-w/2 , -h/2;
        %                 w/2 , -h/2 ;
        %                 k*w/2 , h/2 ;
        %                 -k*w/2 , h/2]*R(theta-pi/2)';
        %
        %             hP=patch(XY(:,1)+p(1),XY(:,2)+p(2),clr);
        %         end
        %
        function obj = RealCar (varargin)
            obj = obj@CtSystem(varargin{:});
            
            %% Retrive parameters for superclass GeneralSystem
            
            parameterPointer = 1;
            hasParameters = length(varargin)-parameterPointer>=0;
            
            while hasParameters
                if (ischar(varargin{parameterPointer}))
                    switch varargin{parameterPointer}
                        case 'dt'
                            obj.dt = varargin{parameterPointer+1};
                            parameterPointer = parameterPointer+2;
                        case 'DownsampleSendCommand'
                            obj.downsampleSendCommand = varargin{parameterPointer+1};
                            parameterPointer = parameterPointer+2;
                        case 'RemoteIp'
                            obj.remoteIp = varargin{parameterPointer+1};
                            parameterPointer = parameterPointer+2;
                        otherwise
                            parameterPointer = parameterPointer+1;
                    end
                else
                    parameterPointer = parameterPointer+1;
                end
                
                hasParameters = length(varargin)-parameterPointer>=0;
            end
            
            %% Ask initial Condition
            if isempty(obj.nx)
                obj.nx=1;
            end
            if isempty(obj.nu)
                obj.nu=2;
            end
            if isempty(obj.ny)
                obj.ny=3;
            end
            
            obj.initialConditions = zeros(obj.nx,1);
            
            obj.f = @(t,x,u)obj.sendCommand(t,u);
            obj.h = @(t,x)obj.measureROS(t);
            
            
            PC_IP = '10.42.0.33';
            BEAGLEBONE_IP = 'http://10.42.0.14';
            
            % Initialise ROS on remote master
            setenv('ROS_MASTER_URI', strcat(BEAGLEBONE_IP, ':11311'))
            setenv('ROS_IP', PC_IP)
            rosinit
            
            % Setup engine control publisher
            obj.engine_pub = rospublisher('/car/actuator_engine_update',rostype.std_msgs_Float64);
            obj.engine_msg = rosmessage(obj.engine_pub);
            
            % Setup servo control publisher
            obj.steering_pub = rospublisher('/car/actuator_steering_update',rostype.std_msgs_Float64);
            obj.steering_msg = rosmessage(obj.steering_pub);
            
            obj.gps_sub = rossubscriber('/car/fix', rostype.sensor_msgs_NavSatFix, @gpsCallback,'BufferSize',1);
            
            obj.imu_sub = rossubscriber('/car/imu_readings', rostype.sensor_msgs_Imu, @imuCallback,'BufferSize',1);
            obj.mag_sub = rossubscriber('/car/mag_readings', rostype.sensor_msgs_MagneticField, @magCallback,'BufferSize',1);
            
            % Initialize origin
            msg_gps = receive(obj.gps_sub);
            obj.lat0 = msg_gps.Latitude;
            obj.lon0 = msg_gps.Longitude;
            
            disp('Initialize heading...');
            yawSum = 0;
            nCompass_samples = 50;
            for i = 1:nCompass_samples
                msg_mag = receive(obj.mag_sub);
                yawSum = yawSum + atan2(...
                    msg_mag.MagneticField_.X,...
                    msg_mag.MagneticField_.Y...
                    );
            end
            obj.yaw0 = yawSum/nCompass_samples;
            
            disp('Initial heading');
            disp(obj.yaw0);
            
            global gps_new_data
            global imu_new_data
            
            gps_new_data = 0;
            imu_new_data = 0;
        end
        
        function fakeDotX = sendCommand(obj,t,u)
            
            %% Send command u(t) to the vehicle
            
            % Update car speed and steering angle
            steeringAngle = u(2);                   % Between -0.35 (left) and 0.35 (right)
            carSpeed   = u(1);                      % Up to 10
            
            
            % Input command failsafe
            if carSpeed > 6
                carSpeed = 6;
            elseif carSpeed < 0
                carSpeed = -20;
            end
            
            steeringValue = 1.463*steeringAngle + sign(steeringAngle)*0.059;

            engineValue = 0.9*carSpeed + 2.543;
            
            if abs(carSpeed) < 0.1
                engineValue = 0;
            end
            
%             % Output failsafe
%             if engineValue > 8
%                 engineValue = 8;
%             end
            
            obj.engine_msg.Data = engineValue-12.5;
            obj.steering_msg.Data = -steeringValue-0.5;
            
            send(obj.engine_pub, obj.engine_msg)
            send(obj.steering_pub, obj.steering_msg)
            
            fakeDotX = 0;
        end
        
        function y = measureROS(obj,t)
            global car_lat
            global car_lon
            global gps_new_data
            
            global imu_data
            global imu_new_data
            
            drawnow;
            
            y = [NaN NaN NaN NaN NaN NaN]';
                
            
            if gps_new_data == 1
                [x1, x2] = latlon2carthesian(obj.lat0, obj.lon0, car_lat, car_lon);
                if t < 0.4
                    obj.lat0 = car_lat;
                    obj.lon0 = car_lon;
                end
                
%                 d = sqrt(x1*x1+x2*x2);
                
%                 fprintf('Coordinates:  \n\t x: %2.2f (m)\t y: %2.2f (m)\t d: %2.2f (m)\n', x1, x2, d);
                y(1:2) = [x1;x2];
                gps_new_data = 0;
            end
            
            if imu_new_data == 1                
                imu_new_data = 0;
%                 y(3) = imu_data.accelerometer(2);   % x acceleration (y IMU's axis)
%                 y(4) = -imu_data.accelerometer(1);   % y acceleration (x IMU's axis)
                y(5) = imu_data.gyroscope(3);       % z angular rate
                
%                 yawMeas = atan2(...                     % yaw angle from magnetometer readings
%                     imu_data.magnetometer(1),...
%                     imu_data.magnetometer(2)...
%                     );
%                 yawEst = obj.stateObserver.x(4);
% 
%                 if abs(yawMeas - yawEst) > 0.6
%                     yaw = yawEst;   % Measurement is an outliser
%                 else
%                     yaw = yawMeas;
%                 end
                
%                 y(6) = yawMeas + 1.6167*pi/180; % Adjust magnetic declination (Renens: 1°37'East)
%                 y(6) = yawMeas;
%                 fprintf('\nNew IMU measurements\n');
            end
            
        end % function
        
    end     % methods
end         % class