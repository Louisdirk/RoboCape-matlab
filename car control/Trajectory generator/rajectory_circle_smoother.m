%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Author: Mathieu Bresciani
% 
% Draw trajectory
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clear all
close all

radius = 1;

R = @(phi) [cos(phi) , -sin(phi);
            sin(phi) , cos(phi)];

nCheckpoints = 5;
p = zeros(nCheckpoints, 2);

p(1,:) = [0, 0];
p(2,:) = [5, 0];
p(3,:) = [7, 2];
p(4,:) = [4, 4];
p(5,:) = [0, 2];

dir(:, nCheckpoints) = [0;0];
for n = 1:(nCheckpoints-1)
    dir(:,n) = points2vectU(p(n,:),p(n+1,:)); % Computes unitary direction vectors.
    l(n) = norm(p(n+1)-p(n));
end

alpha = zeros(nCheckpoints,1);
l2 = zeros(nCheckpoints,1);
alpha(1) = 0;
alpha(nCheckpoints) = 0;
l2(1) = 0;
l2(nCheckpoints) = 0;
psi(1) = vectU2psi([1;0],dir(:,1));
phi(1) = psi(1);

for n = 2:(nCheckpoints-1)
    psi(n) = vectU2psi(dir(:,n-1),dir(:,n));
    alpha(n) = psi2alpha(psi(n));
    phi(n) = wrapTo2Pi(phi(n-1)+psi(n));
    
    l2(n) = radius/tan(alpha(n));
end

for n = 1:(nCheckpoints-1)
    l1(n) = l(n) - l2(n) - l2(n+1);
end

%% Display path

stepSize = 0.01;

lTot = sum(l1) + sum(radius*2*alpha); % Total path length
d = 0:stepSize:lTot;

figure;
plot(p(:,1),p(:,2));
hold all
for n = 2:(nCheckpoints-1)
    center(n,:) = p(n,:)' + l2(n)*dir(:,n) + radius*[-dir(2,n);dir(1,n)];
    plot(radius*cos(0:0.1:2*pi)+center(n,1), radius*sin(0:0.1:2*pi)+center(n,2));
end
plot(center(:,1),center(:,2), '*');
% quiver(p(:,1),p(:,2), dir(1,:)', dir(2,:)');
axis([0 8 0 8])
hold off

%% Generate path and trajectories
% for k = 1:length(d)
%     path(k,:) = d*dirVectU;
%     
% end
