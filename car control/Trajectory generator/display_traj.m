%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Author: Mathieu Bresciani
% 
% Computes time-optimal trajectory with initial and final conditions
% for position and velocity, velocity saturation and maximal acceleration
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clear all
close all

aMax = 2;
vSat = 4;

v0 = 2;
vf = 3;
x0 = 0.5;
xf = 10;

%% Solve optimal time trajectory

% Unreachable vf test
T1 = sqrt(2*(xf-x0)/aMax);
vMax = aMax*T1 + v0;
if vf > vMax
    vf = vMax;  % DIRECT RETURN T2=T3=0 IN FUNCTION!!!
end
    
T2 = 0;
T3a = -(2*vf + 2^(1/2)*(v0^2 + vf^2 - 2*aMax*x0 + 2*aMax*xf)^(1/2))/(2*aMax);
T3b = -(2*vf - 2^(1/2)*(v0^2 + vf^2 - 2*aMax*x0 + 2*aMax*xf)^(1/2))/(2*aMax);

if T3a >= 0
    T3 = T3a;
else
    T3 = T3b;
end

if aMax*T3 + vf <= vSat
    T1 = ((vf - v0) + aMax*T3)/aMax;
else
    T1 = (vSat - v0)/aMax;
    T3 = (vSat - vf)/aMax;
    T2 = -(x0 - xf + (v0 - vSat)^2/(2*aMax) - (vSat - vf)^2/(2*aMax) - (v0*(v0 - vSat))/aMax + (vSat*(vSat - vf))/aMax)/vSat;
end

%% Simulation

t0 = 0;
t1 = t0+T1;
t2 = t1+T2;
t3 = t2+T3;

t = t0:0.01:t3;

vt1 = aMax*T1 + v0;
vt3 = aMax*(T1-T3) + v0;
xt1 = 1/2*aMax*T1^2 + v0*T1 + x0;
xt2 = aMax*T1*T2 + v0*T2 + xt1;
xt3 = -1/2*aMax*T3^2 + vt1*T3 + xt2;

for k = 1:length(t)
    if t(k) <= t1
        tLoc = t(k)-t0;
        a(k) = aMax;
        v(k) = aMax*tLoc + v0;
        x(k) = 1/2*aMax*tLoc^2 + v0*tLoc + x0;
    elseif t(k) <= t2
        tLoc = t(k)-t1;
        a(k) = 0;
        v(k) = vt1;
        x(k) = vt1*tLoc + xt1;
    else
        tLoc = t(k)-t2;
        a(k) = -aMax;
        v(k) = -aMax*tLoc + vt1;
        x(k) = -1/2*aMax*tLoc^2 + vt1*tLoc + xt2;
    end
end

figure;
subplot(3,1,1);
plot(t,a);
subplot(3,1,2);
plot(t,v);
subplot(3,1,3);
plot(t,x);

