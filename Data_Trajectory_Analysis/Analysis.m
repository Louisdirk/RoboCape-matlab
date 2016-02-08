%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Author: Mathieu Bresciani
% Description:
%   This codes extracts data recorded in Virtual Arena and make plots for
%   analysis
%
% Dependencies:
%   - ui_plot_traj.m
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

close all

l = 0.34;
lr = 0.17;

tv = 1.1;
tdelta = 0.2;

%% Extract measurements

mask = isnan(ret{1}.measurements(5,:));
j = 1;
t = ret{1}.time;
for i = 1:length(ret{1}.measurements(5,:))
    if mask(i) == 0
        meas(:,j) = ret{1}.measurements(:,i);
        t_meas(j) = ret{1}.time(i);
        j = j+1;
    end
end

mask_gps = isnan(ret{1}.measurements(1,:));
t_gps = t(~mask_gps);
meas_gps = ret{1}.measurements(:,~mask_gps);
speed_from_gps = zeros(1,length(meas_gps));

for i = 2:length(meas_gps)
    speed_from_gps(i) = sqrt((meas_gps(1,i)-meas_gps(1,i-1))^2 + (meas_gps(2,i)-meas_gps(2,i-1))^2)*5;
end

figure; plot(t_gps,speed_from_gps)
%% State observation evolution
inputs = ret{1}.inputTrajectory;
Ts = t(2)-t(1);

inputs(1,:) = max(inputs(1,:),0);

% Velocity model filter
Fs1 = tf(1,[tv 1]);
Fz1 = c2d(Fs1,Ts);
inputsF(1,:) = filter(Fz1.num{1}, Fz1.den{1},inputs(1,:));

% Steering model filter
Fs2 = tf(1,[tdelta 1]);
Fz2 = c2d(Fs2,Ts);
inputsF(2,:) = filter(Fz2.num{1}, Fz2.den{1},inputs(2,:));


gpsEst = ret{1}.observerStateTrajectory(1:2,:);
vfEst = ret{1}.observerStateTrajectory(3,:);
yawEst = ret{1}.observerStateTrajectory(4,:);
steeringEst = ret{1}.observerStateTrajectory(5,:);

varEst = ret{1}.observerStateTrajectory(6:6:6*5,:);
sigEst = sqrt(varEst);

% Plot trajectory estimation with position measurements in gui
traj = TrajectoryGenerator(p,pv,radius,aCircle,aMax,vSat);
[traj_ref, ~, ~] = traj.getSampledTraj(Ts,t(end));
ui_plot_traj(t, ret{1}.measurements, t, gpsEst, vfEst, yawEst, traj_ref');

figure(2);
plot(t, inputs(1,:), t, inputsF(1,:),t,vfEst);
hold all
plot(t, vfEst+sigEst(3,:), 'g--',t,vfEst-sigEst(3,:), 'g--');
hold off
title('Forward speed');
xlabel('Time (s)');
ylabel('Speed (m/s)');
legend('v_{ref}', 'v_{model}', 'v_{est}', '\sigma_{est}');

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
steeringFromGyro = asin(meas(5,:)*l./vfEst(~mask));

plot(t, inputs(2,:), t, inputsF(2,:), t,unwrap(steeringEst), t_meas, steeringFromGyro);
hold all
plot(t, unwrap(steeringEst+sigEst(5,:)), 'g--', t, unwrap(steeringEst-sigEst(5,:)), 'g--')
hold off
title('Steering angle');
xlabel('Time (s)');
ylabel('Steering angle (rad)');
legend('\delta_{ref}', '\delta_{model}' , '\delta_{est}', 'Value from gyro, speed and geometry', '\sigma_{est}');

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
    'GNSS_x', ...
    'GNSS_y', ...
    'Acc_x', ...
    'Acc_y', ...
    'Gyro_z', ...
    'Compass'};

% for i = 1:yn
%     ax(i) = subplot(yn,1,i);
%     stem(t,inn(i,:));
%     ylabel(yNamesList{i});
% end
j = 0;
for i = [1 2 5]
    j = j +1;
    ax(j) = subplot(3,1,j);
    stem(t,inn(i,:));
    ylabel(yNamesList{i});
end
% xlim([0 50]);
linkaxes(ax,'x');
xlabel('Time (s)');
% subplot(yn,1,1)
% title('Innovation');

%% Path from yaw estimate


posFromYaw_mem = zeros(2,1);
posFromYaw = 0;

vfMeas = vfEst(~mask); % vfEst at "meas" time
yawInt = 3/2*pi;

%
% vfEst = max(inputsF(1,:),0);
%
for i = 2:length(t_meas)
    deltaT = t_meas(i) - t_meas(i-1);
    yawInt(i) = yawInt(i-1) + meas(5,i)*deltaT;
    dx = [
        cos(yawInt(i))*vfMeas(i)*deltaT;          % xDot
        sin(yawInt(i))*vfMeas(i)*deltaT;          % yDot
        ];
    posFromYaw = posFromYaw + dx;
    posFromYaw_mem = [posFromYaw_mem posFromYaw];
    
end

fig = figure;
plot(posFromYaw_mem(1,2:end),posFromYaw_mem(2,2:end));
title('Path from yaw and speed estimates');

dcm_obj = datacursormode(fig);
set(dcm_obj,'UpdateFcn',{@myupdatefcn,t_meas})


%% 
figure;
plot(t,sigEst(4,:))
legend('Yaw std dev');
xlabel('Time (s)');

figure;
plot(t,sigEst(1,:))
legend('xPos sqrt(var)');
xlabel('Time (s)');

figure;
plot(t,sigEst(3,:))
legend('vf sqrt(var)');
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
hl = legend('$e_1$','$e_2$','$z$','$V$');
set(hl, 'Interpreter','latex');
xlabel('Time (s)');
ylabel('Amplitude');

%% Communication quality
IMU_Tics = ~isnan(ret{1}.measurements(5,:));
windowSize = 10;
IMU_Density = conv(double(IMU_Tics), ones(1,windowSize), 'valid')/windowSize;
figure;
subplot(2,1,1);stem(t,IMU_Tics*100,'Color',[0.8 0.8 0.8]) % IMU
hold all
plot(t(1:end-(windowSize-1)),IMU_Density*100,'Color',[0 0 0]);
hold off
ylabel('Q_{imu} (%)');
% xlim([0 50]);

GPS_Tics = ~isnan(ret{1}.measurements(1,:));
GPS_Density = conv(double(GPS_Tics), ones(1,windowSize), 'valid')/(windowSize);
subplot(2,1,2);stem(t,GPS_Tics*100,'Color',[0.8 0.8 0.8]) % GNSS
hold all
plot(t(1:end-(windowSize-1)),min(GPS_Density*100*3.3,100),'Color',[0 0 0]);
hold off
xlabel('Time (s)');
ylabel('Q_{gnss} (%)');
% xlim([0 50]);

figure; histogram(diff(t(IMU_Tics)),30,'Normalization','probability')
figure; histogram(diff(t(GPS_Tics)),10)
