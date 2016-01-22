clear all;
close all;
clc;

load('.\13jan\20sec2.mat')

replaySys = ReplayMeasuramentsCtSystem('TimeLog',ret{1}.time,'MeasurementLog',ret{1}.measurements,'nu',2);
replaySys.controller = ReplayController(ret{1}.time,ret{1}.inputTrajectory);

dt = 0.06;
%%
ny    = 6; % Number outputs (2 for position, 3 for position and heading (in radiant)

model = ModelRealCar(...                  % Car model, state vector: [position x; position y; forward velocity; heading; steering angle]
    'ny',ny,...                           % Number of outputs
    'InitialCondition',0.1*ones(5,1),...  % Initial state vector
    'Parameters',[(1/1.8);(1/0.1);1;1]... % Parameters first order models (xDot = -k*(x-g*u)):  [k velocity; k steering angle; g velocity; g steering angle;]
    );

model.initialConditions(1:2)=[0;0];

%% MODE
% mode 1 > simulation
% mode 2 > real vehicle
mode = 2;
var_gps = (0.1)^2;            % m
var_acc = (0.05)^2;         % m/s^2
var_gyro = (0.1)^2;        % rad/sec
var_mag = (60/180*pi)^2;    % rad

Rsys = diag([var_gps var_gps var_acc var_acc var_gyro var_mag]);
Qsys = diag(([0.01;0.01;0.05;10*pi/180;2*pi/180]).^2);

%%
% At this point, it is possible to use the measurements and control inputs 
% from the experiment to improve the observer (e.g., testing a different
% models or parameters).

replaySys.stateObserver = EkfFilter(DtSystem(model,dt),...
            'StateNoiseMatrix'  , Qsys,... %noise injection
            'OutputNoiseMatrix' , Rsys,... %noise injection
            'InitialCondition' , [0.01*ones(model.nx,1);reshape(eye(model.nx),model.nx^2,1)]);
        
replaySys.stateObserver.innovationFnc =  @(t,z,y)innFnc(t,z,y,...
    0.1,... %Saturation on Position Error
    0.5,... %Saturation on Heading Error
    2);     %Beginning Saturation Time
        
                               
a2 = VirtualArena(replaySys,...
    'ExtraLogs'        ,   MeasurementsLog(model.ny), ...
    'StoppingCriteria'   , @(i,agentsList)i>15,...
    'DiscretizationStep' , dt...
    );

%     'StepPlotFunction'   , @stepPlotFunctionEkf, ...
%     'PlottingStep'       , 0,...

logs2 = a2.run();
clear ret
ret = logs2;
