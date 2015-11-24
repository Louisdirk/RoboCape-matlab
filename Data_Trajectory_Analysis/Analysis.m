
close all
%% Extract measurements

mask = isnan(ret{1}.measurements(1,:));
j = 1;
t = ret{1}.time;
for i = 1:length(ret{1}.measurements(1,:))
    if mask(i) == 0
        meas(:,j) = ret{1}.measurements(:,i);
        t_meas(j) = ret{1}.time(i);
        j = j+1;
    end
end

%% State observation evolution
inputs = ret{1}.inputTrajectory;

Ts = t(2)-t(1);
% Steering model filter
Fs = tf(1,[0.1 1]);
Fz = c2d(Fs,Ts);
inputsF(2,:) = filter(Fz.num{1}, Fz.den{1},inputs(2,:));

% figure; plot(t,inputs(2,:),t,inputsF(2,:))

gpsEst = ret{1}.observerStateTrajectory(1:2,:);
vfEst = ret{1}.observerStateTrajectory(3,:);
yawEst = ret{1}.observerStateTrajectory(4,:);
steeringEst = ret{1}.observerStateTrajectory(5,:);

% Plot trajectory estimation with position measurements in gui
ui_plot_traj(t_meas, meas, t, gpsEst, vfEst, yawEst);

figure(2);
plot(t,vfEst);
title('Estimated speed');
xlabel('Time (s)');
ylabel('Speed (m/s)');

figure(3);
plot(t,yawEst);
hold all
plot(t_meas,unwrap(meas(6,:)),'x');
title('Yaw angle');
xlabel('Time (s)');
ylabel('Yaw angle (rad)');
legend('Estimated', 'Measured');

figure(4);
plot(t,unwrap(steeringEst),t, inputs(2,:), t, inputsF(2,:));
title('Steering angle');
xlabel('Time (s)');
ylabel('Steering angle (rad)');

%% Imu readings
figure(5);
plot(t_meas,meas(3,:),'-*');
hold all;
plot(t_meas,meas(4,:), '-+');
title('Accelerometers');
xlabel('Time (s)');
ylabel('Acceleration (m/s^2)');
legend('X axis', 'Y axis');

figure(6);
plot(t_meas,meas(5,:));
title('Gyroscope');
xlabel('Time (s)');
ylabel('Angular rate (rad/s)');
legend('Z axis');

%% Innovation
yn = size(meas,1);
inn = ret{1}.inn;

figure(7);

yNamesList = {...
    'gpsX', ...
    'gpsY', ...
    'accX', ...
    'accY', ...
    'gyroZ', ...
    'yaw'};

for i = 1:yn
    subplot(yn,1,i)
    plot(t,inn(i,:));
    ylabel(yNamesList{i});
end

xlabel('Time (s)');
subplot(yn,1,1)
title('Innovation');
