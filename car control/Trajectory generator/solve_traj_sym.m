clear all

syms T1 T2 T3 aMax v0 x0 vf xf vSat

vt1 = aMax*T1 + v0;
vt3 = aMax*(T1-T3) + v0;
xt1 = 1/2*aMax*T1^2 + v0*T1 + x0;
xt2 = aMax*T1*T2 + v0*T2 + xt1;
xt3 = -1/2*aMax*T3^2 + vt1*T3 + xt2;

T2_is_null = 0;

if T2_is_null == 1
    %% T2 = 0
    T1 = solve(aMax*T1 - aMax*T3 == vf - v0, T1);
    
    xt3_noT2 = -1/2*aMax*T3^2 + (aMax * T1 + v0)*T3 + 1/2*aMax*T1^2 + T1*v0 + x0;
    
    T3 = solve(xt3_noT2 == xf, T3);
    
else
    %% T2 ~= 0
    T3 = (vSat - vf)/aMax;
    T1 = (vSat - v0)/aMax;
    
    xt3_T2 = -1/2*aMax*T3^2 + (aMax*T1 + v0)*T3 + aMax*T1*T2 + v0*T2 + T1*v0 + 1/2*aMax*T1^2 + x0;
    T2 = solve(xt3_T2 == xf, T2);
end