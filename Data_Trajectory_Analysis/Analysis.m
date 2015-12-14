
close all

l = 0.34;
lr = 0.17;

%% Extract measurements

mask = isnan(ret{1}.measurements(5,:));
j = 1;
t = ret{1}.time;
for i = 1:length(ret{1}.measurements(5,:))
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
inputs(2,:) = inputs(2,:);

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

varEst = ret{1}.observerStateTrajectory(6:6:6*5,:);
sigEst = sqrt(varEst);

% Plot trajectory estimation with position measurements in gui
ui_plot_traj(t_meas, meas, t, gpsEst, vfEst, yawEst);

% x0 = 0;
% y0 = 0;
% k = 1;
% for i = 1:length(t_meas)
%     if not(isnan(meas(1,i)))
%         dxGps(i) = sqrt((x0-meas(1,i))^2 + (y0-meas(2,1))^2);
%         x0 = meas(1,i);
%         y0 = meas(2,1);
%         tGps(k) = t_meas(i);
%         k = k+1;
%     end
% end

% vfGps = dxGps*5; % GPS @ 5Hz
figure(2);
plot(t, inputs(1,:),t,vfEst);
title('Forward speed');
xlabel('Time (s)');
ylabel('Speed (m/s)');

figure(3);
plot(t,yawEst);
hold all
plot(t,yawEst+sigEst(4,:), 'g--',t,yawEst-sigEst(4,:), 'g--')
plot(t_meas,unwrap(meas(6,:)),'x');
title('Yaw angle');
xlabel('Time (s)');
ylabel('Yaw angle (rad)');
legend('Estimated', 'Confidence interval', '', 'Measured');

figure(4);
steeringFromGyro = asin(meas(5,:)*l./1);
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

figure;
plot(t,ret{1}.observerStateTrajectory(24,:))
legend('Yaw covariance');
xlabel('Time (s)');

figure;
plot(t,sqrt(ret{1}.observerStateTrajectory(6,:)))
legend('xPos sqrt(covariance)');
xlabel('Time (s)');

%% Lyapunov variables
figure;
e = [ret{1}.lyapVar(1,:); ret{1}.lyapVar(2,:)];
z = ret{1}.lyapVar(3,:);
V = zeros(1,length(e));
for i = 1:length(t)
    V(i) = 0.5*(e(:,i)'*e(:,i)) + 0.5*z(i)^2;
end
plot(t,ret{1}.lyapVar(1,:),t,ret{1}.lyapVar(2,:),t,ret{1}.lyapVar(3,:),t,V)

title('Lyapunov variables and function');
legend('e1','e2','z',['1/2e' char(39) 'e + 1/2z^2']);
xlabel('Time (s)');