%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Author: Mathieu Bresciani
%
% Draw trajectory
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clear all
close all

radius = 0.5;
aCircle = 0.6;                            % Maximum centripetal acceleration

aMax = 1;                               % Maximum forward acceleration
vSat = 2;                               % Maximal velocity

nCheckpoints = 5;
p = zeros(nCheckpoints, 2);

p(1,:) = [0, 0];
p(2,:) = [-8, 0];
p(3,:) = [-8, -2];
p(4,:) = [-6, -2];
p(5,:) = [-6, 0];

vCircle = sqrt(aCircle*radius);         % Max speed in curve

pv = [0; vCircle; vCircle; vCircle; 0]; % Checkpoint velocities

%%
% clear all
% load('traj2_Osterrechichring_Austria_params');
%%

dir(:, nCheckpoints) = [0;0];
for n = 1:(nCheckpoints-1)
    dir(:,n) = points2vectU(p(n,:),p(n+1,:)); % Computes unitary direction vectors.
    l(n) = norm(p(n+1)-p(n));
end

alpha = zeros(nCheckpoints,1);
l2 = zeros(nCheckpoints,1);
alpha(1) = 0;
l2(1) = 0;
l2(nCheckpoints) = 0;
psi(1) = atan2(dir(2,1),dir(1,1));
phi(1) = psi(1);

for n = 2:(nCheckpoints-1)
    psi(n) = vectU2psi(dir(:,n-1),dir(:,n));
    alpha(n) = psi2alpha(abs(psi(n)));
    phi(n) = wrapTo2Pi(phi(n-1)+psi(n));
    
    l2(n) = abs(radius/tan(alpha(n)));
end

for n = 1:(nCheckpoints-1)
    l1(n) = l(n) - l2(n) - l2(n+1);
end

%% Display path with smoothing circles

figure;
plot(p(:,1),p(:,2));
hold on
for n = 2:(nCheckpoints-1)
    turnDir(n) = sign(det([dir(:,n-1),dir(:,n)]));
    center(n,:) = p(n,:)' - l2(n)*dir(:,n-1) + turnDir(n)*radius*[-dir(2,n-1);dir(1,n-1)];
    plot(radius*cos(0:0.1:2*pi)+center(n,1), radius*sin(0:0.1:2*pi)+center(n,2));
end
plot(center(:,1),center(:,2), '*');
% quiver(p(:,1),p(:,2), dir(1,:)', dir(2,:)');
axis equal
hold off

%% Generate path and trajectories
lTot = sum(l1) + sum(radius*2*alpha);   % Total path length

%% Generate trajectories for straight lines
nSegments = nCheckpoints-1;
nCircles = nCheckpoints-1;
index = 1;
indexC = 2;

for n = 1:2:2*(nSegments)
    p0 = p(index,:) + l2(index)*dir(:,index)';
    pf = p(index+1,:) - l2(index+1)*dir(:,index)';
    path{n} = StraightTrajectory(p0,pf,pv(index),pv(index+1),aMax,vSat);

%     rotation = sign(det([dir(:,index), dir(:,index+1)]));
    path{n+1} = CircularTrajectory(center(indexC,:),radius,phi(indexC-1),alpha(indexC),path{n}.vf,turnDir(indexC));
    
    if indexC < nCircles
        indexC = indexC + 1;
    else
        indexC = 2;
    end
    index = index + 1;
end

%% Generate trajectories for turns
% startAngle(n) = phi(n) - pi/2;
% angularSpeed(n) = segParams(n,4)/radius;
currentPart = 1;
tP = 0;
xStart = 0;
t = 0:0.06:60;
for k = 1:length(t)
    if (path{currentPart}.isEnd() == 1) && (currentPart < (nSegments + nCircles-1))
        currentPart = currentPart + 1;
        tP = t(k);
        xStart = trajProj(1);
%         disp(tP);
    end
    tLoc = t(k) - tP;
    [pSim(k,:), vSim(k,:), aSim(k,:)] = path{currentPart}.getTrajFromTime(tLoc);
    trajProj = path{currentPart}.getTrajProj() + [xStart, 0, 0];
    pSimProj(k) = trajProj(1);
    vSimProj(k) = trajProj(2);
    aSimProj(k) = trajProj(3);
end

%%
figure(1);
hold all
plot(pSim(:,1), pSim(:,2),'.')
% title('Path');
xlabel('x (m)');
ylabel('y (m)');
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
