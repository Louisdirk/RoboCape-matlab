%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Author: Mathieu Bresciani
%
% Draw trajectory
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clear all
close all

radius = 2;
aCircle = 0.3;                            % Maximum centripetal acceleration

aMax = 0.5;                               % Maximum forward acceleration
vSat = 2;                               % Maximal velocity

nCheckpoints = 5;
p = zeros(nCheckpoints, 2);

p(1,:) = [0, 0];
p(2,:) = [0, -3];
p(3,:) = [1, -5];
p(4,:) = [-1, -6];
p(5,:) = [0, -10];
% p(6,:) = [4, 8];
% p(7,:) = [2, 6];
% p(8,:) = [0, 0];

vCircle = sqrt(aCircle*radius);         % Max speed in curve

pv = [0; vCircle; vCircle; vCircle; 0]; % Checkpoint velocities

%%

traj = TrajectoryGenerator(p,pv,radius,aCircle,aMax,vSat);

%% Run simulation

t = 0:0.06:10;
for k = 1:length(t)
    [pSim(k,:), vSim(k,:), aSim(k,:)] = traj.getTrajFromTime(t(k));
    pSimProj(k) = norm(pSim(k,:));
    vSimProj(k) = norm(vSim(k,:));
    aSimProj(k) = norm(aSim(k,:));
end

%%
figure(1);
hold all
plot(pSim(:,1), pSim(:,2),'.')
title('Path');
xlabel('Meters towards east');
ylabel('Meters towards north');
axis equal
figure;
subplot(3,1,1);
plot(t,aSimProj,'.');
ylabel('Acceleration norm');
subplot(3,1,2);
plot(t,vSimProj,'.');
ylabel('Velocity norm');
subplot(3,1,3);
plot(t,pSimProj,'.');
ylabel('Position norm');
