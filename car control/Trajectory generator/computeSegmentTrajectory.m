function [T1,T2,T3,vf] = computeSegmentTrajectory(x0,xf,v0,vf,aMax,vSat)

% Unreachable vf test
T1 = sqrt(2*(xf-x0)/aMax);
vMax = aMax*T1 + v0;
if vf > vMax
    vf = vMax;
    T2 = 0;
    T3 = 0;
    return;
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