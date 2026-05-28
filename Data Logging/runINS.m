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

yaw_ins = zeros(N, 1);

for k = 2:N
    yaw_ins(k) = yaw_ins(k-1) + sensors.gyroB(k, 3) * dt;
end

%% Velocity from accelerometer

vel_ins = zeros(N, 2);   % only x and y needed (flat ground)

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

%% Odometry dead reckoning 
% Uses only wheel encoder velocity + gyroscope yaw
% This should drift less than INS because it integrates once not twice

pos_odo = zeros(N, 2);

for k = 2:N
    psi_k    = yaw_ins(k-1);
    vx_body  = sensors.odoVelB(k, 1);

    pos_odo(k, 1) = pos_odo(k-1, 1) + vx_body * cos(psi_k) * dt;
    pos_odo(k, 2) = pos_odo(k-1, 2) + vx_body * sin(psi_k) * dt;
end

%% plots ( suggested by claude)
figure('Name', 'Stage 2 — INS dead reckoning');

% Trajectory
subplot(2, 2, [1 3]);
plot(gt.posW(:,1), gt.posW(:,2), 'k-',  'LineWidth', 2); hold on;
plot(pos_ins(:,1), pos_ins(:,2), 'r-',  'LineWidth', 1.5);
plot(pos_odo(:,1), pos_odo(:,2), 'm--', 'LineWidth', 1.5);
axis equal; grid on;
xlabel('x (m)'); ylabel('y (m)');
legend('Ground truth', 'INS (accel+gyro)', 'Odometry (encoders+gyro)', 'Location', 'best');
title('Trajectory — Stage 2');

% Position error over time
subplot(2, 2, 2);
err_ins = sqrt((pos_ins(:,1)-gt.posW(:,1)).^2 + (pos_ins(:,2)-gt.posW(:,2)).^2);
err_odo = sqrt((pos_odo(:,1)-gt.posW(:,1)).^2 + (pos_odo(:,2)-gt.posW(:,2)).^2);
plot(sensors.t, err_ins, 'r',  'LineWidth', 1.5); hold on;
plot(sensors.t, err_odo, 'm--','LineWidth', 1.5);
xlabel('time (s)'); ylabel('error (m)'); grid on;
legend('INS error', 'Odometry error');
title('Position error over time');

% Yaw comparison
subplot(2, 2, 4);
plot(sensors.t, gt.eulW(:,3), 'k', 'LineWidth', 2); hold on;
plot(sensors.t, yaw_ins,       'r', 'LineWidth', 1.5);
xlabel('time (s)'); ylabel('yaw (rad)'); grid on;
legend('Ground truth yaw', 'INS yaw estimate');
title('Yaw angle');

