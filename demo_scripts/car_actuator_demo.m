PC_IP = '10.42.0.33';
BEAGLEBONE_IP = 'http://10.42.0.14';

% Initialise ROS on remote master
setenv('ROS_MASTER_URI', strcat(BEAGLEBONE_IP, ':11311'))
setenv('ROS_IP', PC_IP)
rosinit

engine_pub = rospublisher('/car/actuator_engine_update',rostype.std_msgs_Float64)
engine_msg = rosmessage(engine_pub);

steering_pub = rospublisher('/car/actuator_steering_update',rostype.std_msgs_Float64)
steering_msg = rosmessage(steering_pub);

while 1
    for k=[0,10]
        engine_msg.Data = k * 1 - 5;
        steering_msg.Data = k / 5 - 1;

        send(engine_pub, engine_msg)
        send(steering_pub, steering_msg)

        pause(1)
    end
end

rosshutdown
