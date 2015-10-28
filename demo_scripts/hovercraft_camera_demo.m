close all; clear all; clc;

PC_IP = '192.168.7.1';
BEAGLEBONE_IP = 'http://192.168.7.2';

% Initialise ROS on remote master
setenv('ROS_MASTER_URI', strcat(BEAGLEBONE_IP, ':11311'))
setenv('ROS_IP', PC_IP)
rosinit

sub = rossubscriber('/hovercraft/usb_cam/image_raw', rostype.sensor_msgs_Image);

fig_msec = figure; hold on;
fig_img = figure; hold on;
i = 0; index = []; delta_msec = [];
while 1
    msg = receive(sub);
    time = rostime('now');

    index = [index i];
    delta_msec = [delta_msec, (time.Sec - msg.Header.Stamp.Sec) * 1e3 + (time.Nsec - msg.Header.Stamp.Nsec) / 1e6 ];

    figure(fig_msec);
    plot(index, delta_msec, 'b');

    i = i + 1;

    img = readImage(msg);
    figure(fig_img);
    imshow(img);
end
