
close all

l = 0.34;
lr = 0.17;

%% Extract measurements

mask = isnan(ret{1}.measurements(1,:));
j = 1;
t = ret{1}.time;
for i = 1:length(ret{1}.measurements(1,:))
    if mask(i) == 0
        meas(:,j) = ret{1}.measurements(:,i);
        t_meas(j) = ret{1}.time(i);
        if meas(1:2,j) == ret{1}.observerStateTrajectory(1:2,i)
            meas(1:2,j) = [NaN;NaN];
        end
        j = j+1;
    end
end

%% State observation evolution
inputs = ret{1}.inputTrajectory;
inputs(2,:) = 0.5*inputs(2,:);

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
steeringFromGyro = asin(meas(5,:)*l./0.8);
plot(t, inputs(2,:), t, inputsF(2,:), t_meas, steeringFromGyro, t,unwrap(steeringEst));
title('Steering angle');
xlabel('Time (s)');
ylabel('Steering angle (rad)');
legend('Desired', '1st order model', 'Value from gyro, speed and geometry', 'State estimate');

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
    stem(t,inn(i,:));
    ylabel(yNamesList{i});
end

xlabel('Time (s)');
subplot(yn,1,1)
title('Innovation');

%% Path from yaw estimate
posFromYaw_mem = zeros(2,1);
posFromYaw = 0;
for i = 1:length(t)
    dx = [
        cos(yawEst(i))*vfEst(i)*Ts;          % xDot
        sin(yawEst(i))*vfEst(i)*Ts;          % yDot
        ];
    posFromYaw = posFromYaw + dx;
    posFromYaw_mem = [posFromYaw_mem posFromYaw];
end

figure;
plot(posFromYaw_mem(1,2:end),posFromYaw_mem(2,2:end));