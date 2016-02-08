%% Model only
l = 0.34;
lr = 0.17;

tv = 1.8;
gv = 1;
td = 0.1;
gd = 1;
        
beta = @(x)atan((lr/l)*tan(x(5)));

inputs = ret{1}.inputTrajectory;

Ts = t(2)-t(1);
% Steering model filter
Fs1 = tf(gd,[td 1]);
Fz1 = c2d(Fs1,Ts);
inputsF(2,:) = filter(Fz1.num{1}, Fz1.den{1},inputs(2,:));

% Velocity model filter
Fs2 = tf(gv,[tv 1]);
Fz2 = c2d(Fs2,Ts);
inputsF(1,:) = filter(Fz2.num{1}, Fz2.den{1},inputs(1,:));

x = zeros(5,1);
x_mem = x;
for i = 1:length(t)
    u = inputs(:,i);
    dxState = [
        cos(x(4)+beta(x))*x(3)*Ts;          % xDot
        sin(x(4)+beta(x))*x(3)*Ts;          % yDot
        0 ;                                 % vDot  compute with "filter"
        (x(3)/l)*cos(beta(x))*tan(x(5))*Ts; % phiDot
        0;                                  % deltaDot compute with "filter"
        ];
    x = x+dxState;
    x(3) = inputsF(1,i);
    x(5) = inputsF(2,i);
    x_mem = [x_mem x];
end

figure; 
plot(x_mem(1,:),x_mem(2,:));