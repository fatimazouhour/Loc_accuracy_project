%% Basic Parameters

T= 60 ; % simulation run time 

% sim time for testing sensors = 60 
% sim time for ann = 3600
dt =  0.05; % sampling time is 1s & CoppeliaSim's default physics dt is 50 ms; matching it avoids
%     aliasing between physics steps and sensor reads.

%% connection with CoppeliaSim

ipAddress = '127.0.0.1' ; % local host
port= 19999; % default Api

%% from manufacturers data cheat 
wheelRadius= 0.0975; %m
wheelBase = 0.381 ; % dist between both wheels

%% Trajectory parameters
% we chose to have a circular path ( parameters were chosen arbitrarily) 
radius= 1;
speed = 0.2;

%% artificial sensor noise

noise_accel= 0.003; % consistent with Mpu-9150 datacheat mentioned in the paper
noise_gyro= 0.005; % ...  
noise_odo_x = 0.02;   % the paper reflects the REAL pioneer 
noise_odo_y= 0.02;
noise_gps = 0.3161; %consistent with  uBlox NEO-6m datasheet

% We CHOOSE not to put constant biases at this stage, to stay
%faithful to what the paper actually did for the simulation test
g= 9.81;

%% output data collection 
dataDir= './data';
dataFile= 'sensor.mat';

%% usingg artificial sensor file
%

%these are to be used for the functions later on ,
% we pack every thing as one struct so it can be passed as a sigle argument
% later on 
parameters.dt= dt;
parameters.T            = T;
parameters.g            = g;
parameters.wheelRadius  = wheelRadius;
parameters.wheelBase    = wheelBase;
parameters.noise_accel  = noise_accel;
parameters.noise_gyro   = noise_gyro;
parameters.noise_gps    = noise_gps;
parameters.noise_odo_x  = noise_odo_x;
parameters.noise_odo_y  = noise_odo_y;
parameters.dataDir      = dataDir;
parameters.dataFile     = dataFile;
