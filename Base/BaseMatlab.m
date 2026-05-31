clc;
clear;
close all;

% --- SET THE RANDOM SEED ---
rng(0); 

% --- COPPELIASIM SETUP ---
% uncomment/ comment the needed paths

%% HUSSEIN

%addpath('/home/husain5/Documents/CoppeliaSim/CoppeliaSim_Edu_V4_10_0_rev0_Ubuntu22_04/programming/zmqRemoteApi/clients/matlab');
%rehash toolboxcache;
%%  FATIMA

addpath(genpath('C:\Program Files\CoppeliaRobotics\CoppeliaSimEdu\programming\zmqRemoteApi\clients\matlab'));

%%
client = RemoteAPIClient();
sim = client.require('sim');

robot = sim.getObject('/PioneerP3DX');
leftMotor = sim.getObject('/PioneerP3DX/leftMotor');
rightMotor = sim.getObject('/PioneerP3DX/rightMotor');

% Simulation Parameters
dt = 0.05;
simulationiteration = 30; 
total_steps = ceil(simulationiteration / dt); 

% ENABLE SYNCHRONOUS MODE for perfect timing
client.setStepping(true);
sim.startSimulation();

% Pioneer P3-DX specifications
wheelRadius = 0.0975; % meters
wheelBase = 0.381;    % meters

% --- DYNAMIC INITIAL CONDITIONS ---
initial_pose = sim.getObjectPosition(robot, sim.handle_world);
initial_ori = sim.getObjectOrientation(robot, sim.handle_world);


% Odometry Init
x_odom_previous = initial_pose{1};
y_odom_previous = initial_pose{2};
theta_odom_previous = initial_ori{3};
initial_heading = initial_ori{3};

% INS Init
x_ins_previous = initial_pose{1};
y_ins_previous = initial_pose{2};
vx_ins_previous = 0; 
vy_ins_previous = 0;
theta_ins_previous = initial_ori{3}; 
yaw_previous = 0;

% --- IMU CALIBRATION (Zeroing the gravity bias) ---
% Read the stationary accelerometer values before the robot moves
try
    bias_accelX = sim.getFloatSignal('accelX');
    bias_accelY = sim.getFloatSignal('accelY');
catch
    bias_accelX = 0; 
    bias_accelY = 0;
end

% Artificial sensor noise
noise_accel = 0.003;  % m/s^2
noise_gyro = 0.0107;  % rad/s
noise_odo_vx = 0.3317;% m/s 
noise_odo_vy = 0.214; % m/s
noise_gps = 0.3162;   % meters

% Slippage Parameters
slip_probability = 0.05; 

% --- Data Logging Arrays ---
time_hist = zeros(1, total_steps);
true_x_hist = zeros(1, total_steps);
true_y_hist = zeros(1, total_steps);
gps_x_hist = zeros(1, total_steps);
gps_y_hist = zeros(1, total_steps);
odom_x_hist = zeros(1, total_steps);
odom_y_hist = zeros(1, total_steps);
odom_v_hist= zeros(1,total_steps);
odom_w_hist= zeros(1,total_steps);
ins_x_hist = zeros(1, total_steps);
ins_y_hist = zeros(1, total_steps);

accelX_hist = zeros(1, total_steps);
accelY_hist = zeros(1, total_steps);
gyro_r_hist = zeros(1, total_steps);

previousleftEncoder = sim.getJointPosition(leftMotor);
previousrightEncoder = sim.getJointPosition(rightMotor);

step_idx = 1;

while sim.getSimulationTime() < simulationiteration
    current_time = sim.getSimulationTime();
    
    %======================================================================
    %==========================GROUND TRUTH================================
    %======================================================================
    true_pos = sim.getObjectPosition(robot, sim.handle_world);
    true_X = true_pos{1}; 
    true_Y = true_pos{2};
    

    %======================================================================
    %================================GPS===================================
    %======================================================================
    %--------------------------POSITION FROM GPS---------------------------
    gpsX = true_X + noise_gps * randn();
    gpsY = true_Y + noise_gps * randn();
    

    %======================================================================
    %==============================GYROSCOPE===============================
    %======================================================================
    try
        raw_roll = sim.getFloatSignal('roll');
        raw_pitch = sim.getFloatSignal('pitch');
        raw_yaw = sim.getFloatSignal('yaw');
    catch
        raw_roll = 0; raw_pitch = 0; raw_yaw = 0;
    end
    
    gyro_p = raw_roll + noise_gyro * randn();
    gyro_q = raw_pitch + noise_gyro * randn();
    gyro_r = raw_yaw + noise_gyro * randn(); 


    %======================================================================
    %============================ACCELEROMETER=============================
    %======================================================================
    try
        raw_accelX = sim.getFloatSignal('accelX');
        raw_accelY = sim.getFloatSignal('accelY');
        raw_accelZ = sim.getFloatSignal('accelZ');
    catch
        raw_accelX = 0; raw_accelY = 0; raw_accelZ = 0;
    end
    
    % Subtract the stationary bias to calibrate the IMU
    %accelX = raw_accelX + noise_accel * randn();  
    %accelY = raw_accelY + noise_accel * randn();

    %wasnt subtracted technically
   accelX = (raw_accelX - bias_accelX) + noise_accel * randn();
   accelY = (raw_accelY - bias_accelY) + noise_accel * randn();
   accelZ = raw_accelZ + noise_accel * randn();
   accel = [accelX, accelY, accelZ];

    %--------------------------- INS ALGORITHM --------------------------
    theta_ins_new = theta_ins_previous - (gyro_r * dt);
    
    accel_body_x = accel(1); % Sensor's X is pointing Forward
    accel_body_y = -accel(2); % Sensor's Y is pointing Lateral
    
    accel_global_x = accel_body_x * cos(theta_ins_new) - accel_body_y * sin(theta_ins_new);
    accel_global_y = accel_body_x * sin(theta_ins_new) + accel_body_y * cos(theta_ins_new);
    
    vx_ins_new = vx_ins_previous + (accel_global_x * dt);
    vy_ins_new = vy_ins_previous + (accel_global_y * dt);
    
    x_ins_new = x_ins_previous + (vx_ins_new * dt);
    y_ins_new = y_ins_previous + (vy_ins_new * dt);


    %======================================================================
    %=============================ODOMETER=================================
    %======================================================================
    newleftEncoder = sim.getJointPosition(leftMotor);
    newrightEncoder = sim.getJointPosition(rightMotor);

    raw_delta_left = newleftEncoder - previousleftEncoder;
    raw_delta_right = newrightEncoder - previousrightEncoder;
    deltaleft = atan2(sin(raw_delta_left), cos(raw_delta_left));
    deltaright = atan2(sin(raw_delta_right), cos(raw_delta_right));

    distanceleft = wheelRadius * deltaleft;
    distanceright = wheelRadius * deltaright;
    
    if rand() < slip_probability
        slip_inflation_L = 1.2 + (0.3 * rand()); 
        distanceleft = distanceleft * slip_inflation_L;
    end
    if rand() < slip_probability
        slip_inflation_R = 1.2 + (0.3 * rand()); 
        distanceright = distanceright * slip_inflation_R;
    end

    distance = (distanceright + distanceleft) / 2;
    deltatheta = (distanceright - distanceleft) / wheelBase;

    theta_odom_new = theta_odom_previous + deltatheta;

    v_body_x = distance / dt; 
    v_body_y = 0;             

    v_body_x_noisy = v_body_x + noise_odo_vx * randn();
    v_body_y_noisy = v_body_y + noise_odo_vy * randn();

    vx_global = v_body_x_noisy * cos(theta_odom_new) - v_body_y_noisy * sin(theta_odom_new);
    vy_global = v_body_x_noisy * sin(theta_odom_new) + v_body_y_noisy * cos(theta_odom_new);

    x_odom_new = x_odom_previous + vx_global * dt;
    y_odom_new = y_odom_previous + vy_global * dt;

    % --- Data Logging ---
    time_hist(step_idx) = current_time;
    true_x_hist(step_idx) = true_X;
    true_y_hist(step_idx) = true_Y;
    gps_x_hist(step_idx) = gpsX;
    gps_y_hist(step_idx) = gpsY;
    odom_x_hist(step_idx) = x_odom_new;
    odom_y_hist(step_idx) = y_odom_new;
    odom_v_hist(step_idx)= v_body_x_noisy;
    odom_w_hist(step_idx) = deltatheta/dt;
   
    ins_x_hist(step_idx) = x_ins_new;
    ins_y_hist(step_idx) = y_ins_new;

    accelX_hist(step_idx) = accelX;   % BODY-frame accel x (after noise)
    accelY_hist(step_idx) = accelY;   % BODY-frame accel y (after noise)
    gyro_r_hist(step_idx) = gyro_r;   % yaw RATE (rad/s

    % --- State Updates ---
    previousrightEncoder = newrightEncoder;
    previousleftEncoder = newleftEncoder;
    x_odom_previous = x_odom_new;
    y_odom_previous = y_odom_new;
    theta_odom_previous = theta_odom_new;
    
    x_ins_previous = x_ins_new;
    y_ins_previous = y_ins_new;
    vx_ins_previous = vx_ins_new;
    vy_ins_previous = vy_ins_new;
    theta_ins_previous = theta_ins_new;


    sim.setJointTargetVelocity(leftMotor, 5.9);
    sim.setJointTargetVelocity(rightMotor, 5.0);

    client.step(); 
    step_idx = step_idx + 1;
end

% --- CLEANUP ---
sim.setJointTargetVelocity(leftMotor, 0);
sim.setJointTargetVelocity(rightMotor, 0);
sim.stopSimulation();

% --- Trim Unallocated Zeros ---
time_hist = time_hist(1:step_idx-1);
true_x_hist = true_x_hist(1:step_idx-1);
true_y_hist = true_y_hist(1:step_idx-1);
gps_x_hist = gps_x_hist(1:step_idx-1);
gps_y_hist = gps_y_hist(1:step_idx-1);
odom_x_hist = odom_x_hist(1:step_idx-1);
odom_y_hist = odom_y_hist(1:step_idx-1);
odom_v_hist=odom_v_hist(1:step_idx-1);
odom_w_hist=odom_w_hist(1:step_idx-1);
ins_x_hist = ins_x_hist(1:step_idx-1);
ins_y_hist = ins_y_hist(1:step_idx-1);

accelX_hist = accelX_hist(1:step_idx-1);
accelY_hist = accelY_hist(1:step_idx-1);
gyro_r_hist = gyro_r_hist(1:step_idx-1);

% --- Successive Regression (Savitzky-Golay Filter) ---
window_size = 50; 
gps_x_successive = smoothdata(gps_x_hist, 'sgolay', window_size);
gps_y_successive = smoothdata(gps_y_hist, 'sgolay', window_size);

% --- Plotting Results ---
figure('Name', 'Robot Localization: Raw Sensor Data', 'NumberTitle', 'off');
hold on; grid on;

% 1. True Path (Black)
plot(true_x_hist, true_y_hist, 'k-', 'LineWidth', 2); 
% 2. Odometry (Blue Dashed)
plot(odom_x_hist, odom_y_hist, 'b--', 'LineWidth', 1.5); 
% 3. GPS Raw (Red Dots)
scatter(gps_x_hist, gps_y_hist, 10, 'r', 'filled', 'MarkerFaceAlpha', 0.5); 
% 4. GPS Smoothed (Green Line)
plot(gps_x_successive, gps_y_successive, 'g-', 'LineWidth', 3); 
% 5. INS (Cyan Line)
plot(ins_x_hist, ins_y_hist, 'c-', 'LineWidth', 1.5); 

xlabel('East Position X (meters)');
ylabel('North Position Y (meters)');
title('Raw Independent Data: GPS vs. Odometry vs. INS');

% Legend explicitly matching the order of the plot commands above
legend('True Path', 'Odometry (Drifting)', 'GPS Raw (Noisy)', 'GPS Smoothed (Savitzky-Golay)', 'INS (Calibrated Double Integration)');
axis equal;

%% save all data to be usable
save('kf_input.mat', 'time_hist', 'true_x_hist', 'true_y_hist','gps_x_hist', 'gps_y_hist', 'accelX_hist', 'accelY_hist', 'gyro_r_hist', 'odom_w_hist','odom_v_hist','initial_heading', ...
     'odom_x_hist', 'odom_y_hist', 'ins_x_hist', 'ins_y_hist');