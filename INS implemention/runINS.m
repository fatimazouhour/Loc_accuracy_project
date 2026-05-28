clear 
close all
clc

addpath('..');
config;
load(fullfile(dataDir, dataFile));


N  = numel(sensors.t);
dt = parameters.dt;
g  = parameters.g;

%% 1. Orientation from gyroscope (integrate yaw rate)

% Robot is already moving at k=1 (3 s lead-in), so integrators MUST NOT start at zero.
yaw_ins = zeros(N,1);
vel_ins = zeros(N,2);
pos_ins = zeros(N,2);
pos_odo = zeros(N,2);

yaw_ins(1)   = gt.eulW(1,3);        
vel_ins(1,:) = gt.linVelW(1,1:2);   
pos_ins(1,:) = gt.posW(1,1:2);      
pos_odo(1,:) = gt.posW(1,1:2);      
%% 
for k = 2:N
    psi = yaw_ins(k-1);

    % standard ZYX rotation matrix, yaw only (flat ground)
    % replaces paper eq.7 which contains a typo — see project log 5.5
    cp = cos(psi);
    sp = sin(psi);
    R = [cp,-sp,0;sp,cp,0;0,0,1];

    % specific force in body frame
    f_b = sensors.accelB(k, :)';

    % subtract gravity in body frame BEFORE rotating
    % accelB z ≈ +g when stationary (z-flip applied in readSensors)
    f_b(3) = f_b(3) - g;

    % Rotate to world frame
    a_world = R * f_b;

    % Integrate: only x and y are useful for 2D localization
    vel_ins(k, 1) = vel_ins(k-1, 1) + a_world(1) * dt;
    vel_ins(k, 2) = vel_ins(k-1, 2) + a_world(2) * dt;
end

%% position from velocity (integrate)
pos_ins = zeros(N, 2);

for k = 2:N
    pos_ins(k, 1) = pos_ins(k-1, 1) + vel_ins(k-1, 1) * dt;
    pos_ins(k, 2) = pos_ins(k-1, 2) + vel_ins(k-1, 2) * dt;
end

%% Odometry 
% uses only wheel encoder velocity + gyroscope yaw
% This should drift less than INS because it integrates once not twice

pos_odo = zeros(N, 2);

for k = 2:N
    psi_k    = yaw_ins(k-1);
    vx_body  = sensors.odoVelB(k, 1);

    pos_odo(k, 1) = pos_odo(k-1, 1) + vx_body * cos(psi_k) * dt;
    pos_odo(k, 2) = pos_odo(k-1, 2) + vx_body * sin(psi_k) * dt;
end


