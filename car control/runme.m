rosshutdown
clc; close all; clear all;

addpath('./utils')

dt       = 0.06;
epsilon  = [0.2;0];


%% DESIRED TRAJECTORY
loopTime = 8;
pathType = 1;
scaleSize = 4; % Increase for bigger shapes
switch pathType
    case 1
        pd       = @(t)                   [  1.4*cos(0.5*2*pi/loopTime*t)    ; 0.5*sin(2*pi/loopTime*t)]*0.9*scaleSize - [5 ;0];
        pdDot    = @(t) (2*pi/loopTime)  *[ -1.4*0.5*sin(0.5*2*pi/loopTime*t)  ; 0.5*cos(2*pi/loopTime*t)]*0.9*scaleSize;
        pdDDot   = @(t) (2*pi/loopTime)^2*[ -1.4*0.5^2*cos(0.5*2*pi/loopTime*t);-0.5*sin(2*pi/loopTime*t)]*0.9*scaleSize;
    case 2
        pd       = @(t) [1 + t ; 1 + t];
        pdDot    = @(t) [1 ; 1];
        pdDDot   = @(t) [0 ; 0];
    case 3
        pd       = @(t) [1 + 0.1*t^3 ; 1 + t^2];
        pdDot    = @(t) [0.1*3*t^2 ; 2*t];
        pdDDot   = @(t) [0.1*6*t ; 2];
end

%% VEHICLE

ny    = 6; % Number outputs (2 for position, 3 for position and heading (in radiant)

model = ModelRealCar(...                  % Car model, state vector: [position x; position y; forward velocity; heading; steering angle]
    'ny',ny,...                           % Number of outputs
    'InitialCondition',0.1*ones(5,1),...  % Initial state vector
    'Parameters',[(1/2);(1/0.1);1;1]... % Parameters first order models (xDot = -k*(x-g*u)):  [k velocity; k steering angle; g velocity; g steering angle;]
    );

model.initialConditions(1:2)=[0;0];
model.initialConditions(4) = pi/2;


%% MODE
% mode 1 > simulation
% mode 2 > real vehicle
% mode 3 > noisy model (simulation)
mode = 3;
var_gps = (0.1)^2;         % m
var_acc = (0.05)^2;         % m/s^2
var_gyro = (0.01)^2;        % rad/sec
var_mag = (60/180*pi)^2;    % rad

% EFK analysis 17nov2015
% var_gps = (3)^2;            % m
% var_acc = 200e-6;           % m/s^2
% var_gyro = 8e-6;            % rad/sec
% var_mag = (60/180*pi)^2;    % rad
% Rnoise = diag([var_gps var_gps var_acc var_acc var_gyro var_mag]);
% Qnoise = diag(([0.5;0.5;0.2;10*pi/180;5*pi/180]).^2);

Rnoise = diag([var_gps var_gps var_acc var_acc var_gyro var_mag]);
Qnoise = diag(([0.01;0.01;0.5;5*pi/180;2*pi/180]).^2);

switch mode
    case 1
        sys = model;
        extraVAParams  = {'RealTime' ,0};
    case 2
        car   = RealCar('ny',ny,'dt',dt);
        car.sendCommand(0,[0 0]);
        sys = car;
        extraVAParams  = {'RealTime' ,1,'Integrator',EulerForward()};
    case 3
        noisyModel = NoisyModelRealBuggy(Qnoise,Rnoise,...                  % Car model, state vector: [position x; position y; forward velocity; heading; steering angle]
            'ny',ny,...                           % Number of outputs
            'InitialCondition',0.1*ones(5,1),...  % Initial state vector
            'Parameters',[(1/2);(1/0.1);1;1]... % Parameters first order models (xDot = -k*(x-g*u)):  [k velocity; k steering angle; g velocity; g steering angle;]
            );
        noisyModel.initialConditions(1:2)=[0;0];
        noisyModel.initialConditions(4) = pi/2;
        sys = noisyModel;
        extraVAParams  = {'RealTime' ,0,EulerForward()};
   
        
end


%% CONTROLLER

cdcController = CarController2(...
    'Epsilon',epsilon,...
    'pd',pd,'pdDot',pdDot,'pdDDot',pdDDot,... % Desired trajectory
    'lr',model.lr,'l',model.l,...                 % lr =  length back wheel to center of mass; l = length back wheel to front wheel
    'Ke',1,'kxi',2, ...                        % Gains of the controller (higher values >> mode aggressive)
    'kv',1/2,'kd',1/0.1, ...
    'u1sat',2,'u2sat',0.25 ...
    );
% old: 'Ke',2,'kxi',2 ... 
% sys.controller =  NewCdcControllerAdapter(cdcController, model,100,50*89*pi/180);
sys.controller = cdcController;
% sys.controller =  NewCdcControllerAdapter(cdcController, model,2,50*0.25*pi/180);

% Open loop control
% InlineController(@(t,x)[speed; steeringAngle])
% sys.controller = InlineController(@(t,x)[1;0.25*1]);

% sys.controller = InlineController(@(t,x) clothoidPath(t));
% sys.controller = KeyboardController();
extraLogs = {};
if mode ~= 1
%% STATE OBSERVER
P0 = diag([(1)^2 (1)^2 (0.01)^2 (180*pi/180)^2 (2*pi/180)^2]);

sys.stateObserver = EkfFilter(DtSystem(model,dt),...
    'InitialCondition' , [0.01*ones(model.nx,1);reshape(P0,model.nx^2,1)],...
    'StateNoiseMatrix' , Qnoise...
    );

% sys.stateObserver.initialConditions(4) = pi/2;
sys.stateObserver.initialConditions(4) = sys.yaw0;

% if ny == 2
    sys.stateObserver.Rekf  =  Rnoise;
% else
%     sys.stateObserver.Rekf  = diag(([0.01;0.01;10*pi/2]/3).^2) ; %<================
% end
sys.stateObserver.innovationFnc =  @(t,z,y)innFnc(t,z,y,...
    0.1,... %Saturation on Position Error
    0.5,... %Saturation on Heading Error
    2);     %Beginning Saturation Time
extraLogs = {InlineLog('inn',@(t,a,varargin)a.stateObserver.lastInnovation),extraLogs{:}};
else
 model.h=@(t,x)x;    
 ny = model.nx;
end
extraLogs = {MeasurementsLog(ny),extraLogs{:}};

%% RUN SIMULATION
% extraLogs = {InlineLog('lyapVar',@(t,a,varargin)a.controller.originalController.getLyapunovVariable(t,a.stateObserver.x(1:5)),'Initialization',zeros(3,1))};
extraLogs = {InlineLog('lyapVar',@(t,a,varargin)a.controller.lastE,'Initialization',zeros(3,1)),extraLogs{:}};
% extraLogs= {};

a = VirtualArena(sys,...
    'StoppingCriteria'   ,@(t,as)t>20,...
    'StepPlotFunction'   ,@(agentsList,hist,plot_handles,i)stepPlotFunction(agentsList,hist,plot_handles,i,pd,model), ...
    'DiscretizationStep' ,dt,...
    'ExtraLogs'          ,extraLogs,...
    'PlottingStep'       ,1,...
    extraVAParams{:}); % Since we are using a real system

simTic = tic;

%profile on
ret = a.run();
if mode == 2 % If real car...
    car.sendCommand(0,[0 0]);
end
simTime = toc(simTic);
fprintf('Sim Time : %d',simTime);
%profile viewer

rosshutdown
%% Post plot
return
figure
explodePlot(ret{1}.time, ret{1}.inputTrajectory,'u')

figure
explodePlot(ret{1}.time,  ret{1}.inn,'inn')
figure
%explodePlot(ret{1}.time,  ret{1}.lyapVar,'lyap var')

if mode == 1
    estErr = ret{1}.stateTrajectory-ret{1}.observerStateTrajectory(1:model.nx,:);
    
    figure
    explodePlot(ret{1}.time, estErr,'err x')
end

figure
explodePlot(ret{1}.time, ret{1}.observerStateTrajectory(1:model.nx,:),' x est')
subplot(5,1,1);
hold on
plot(ret{1}.time,ret{1}.measurements(1,:),'o');
subplot(5,1,2);
hold on
plot(ret{1}.time,ret{1}.measurements(2,:),'o');
subplot(5,1,4);
hold on
plot(ret{1}.time,ret{1}.measurements(3,:),'o');

figure
m=ret{1}.measurements
a=ret{1}.observerStateTrajectory(1:2,:)
plot(m(1,:),m(2,:),'o')
hold on
plot(a(1,:),a(2,:))

