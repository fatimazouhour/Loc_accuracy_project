clc;
clear;

addpath('/home/husain5/Documents/CoppeliaSim/CoppeliaSim_Edu_V4_10_0_rev0_Ubuntu22_04/programming/zmqRemoteApi/clients/matlab');
rehash toolboxcache;

client = RemoteAPIClient();
sim = client.require('sim');

robot = sim.getObject('/PioneerP3DX');
leftMotor = sim.getObject('/PioneerP3DX/leftMotor');
rightMotor = sim.getObject('/PioneerP3DX/rightMotor');

%Simulation Parameter
dt =  0.05;
simulationiteration = 50;
sim.startSimulation();

%from manufacturers data sheet 
wheelRadius= 0.0975; %m
wheelBase = 0.381 ; % dist between both wheels
x_odom_previous = 0;
y_odom_previous = 0;
theta_odom_previous = 0;

% artificial sensor noise
noise_accel= 0.003; % consistent with Mpu-9150 data sheet mentioned in the paper
noise_gyro= 0.0107; %  rad/s
noise_odo_vx = 0.3317;   % the paper reflects the REAL pioneer 
noise_odo_vy= 0.214;
noise_gps = 0.3162; %consistent with  uBlox NEO-6m datasheet

%Recieving the left and right encoder values from the simulation
previousleftEncoder = sim.getJointPosition(leftMotor);
previousrightEncoder = sim.getJointPosition(rightMotor);

%This for loops runs infinitly until we stop the matlab execution manually
%to stop printing data and from the simulation to make the robot stop
%running
while sim.getSimulationTime() < simulationiteration



    %---------------------------GPS----------------------------
    % Getting the Position using GPS from CoppeliaSim
    gpsX = sim.getFloatSignal('gpsX') + noise_gps*randn();
    gpsY = sim.getFloatSignal('gpsY') + noise_gps*randn();
    gpsZ = sim.getFloatSignal('gpsZ') + noise_gps*randn();
    gps = [gpsX gpsY gpsZ];


    
    %-------------------------GYROSCOPE-------------------------
    %Getting the angles using Gyroscope from CoppeliaSim 
    roll = sim.getFloatSignal('roll') + noise_gyro*randn();
    pitch = sim.getFloatSignal('pitch') + noise_gyro*randn();
    yaw = sim.getFloatSignal('yaw') + noise_gyro*randn();
    angle = [roll,pitch,yaw];



    %------------------------ACCELEROMETER-----------------------
    % Getting the acceleration using the Accelerometer from CoppeliaSim
    accelX = sim.getFloatSignal('accelX') + noise_accel*randn();
    accelY = sim.getFloatSignal('accelY') + noise_accel*randn();
    accelZ = sim.getFloatSignal('accelZ') + noise_accel*randn();
    accel = [accelX, accelY, accelZ];



    %---------------------------ODOMETER--------------------------
    % Getting the new left and right angles in radian using leftmotor and rightmore from Coppeliasim 
    newleftEncoder = sim.getJointPosition(leftMotor);
    newrightEncoder = sim.getJointPosition(rightMotor);

    %Caculating the delta leftangle and delta rightangle
    rawdeltaleft = newleftEncoder - previousleftEncoder;
    rawdeltaright = newrightEncoder - previousrightEncoder;
    deltaleft = atan2(sin(rawdeltaleft),cos(rawdeltaleft));
    deltaright = atan2(sin(rawdeltaright),cos(rawdeltaright));

    %Determining the distance covered by each wheel
    distanceleft = wheelRadius*deltaleft;
    distanceright = wheelRadius*deltaright;

    %Caclulating the average distance and deltatheta w.r.t the center
    distance = (distanceright + distanceleft)/2;
    deltatheta = (distanceright - distanceleft)/wheelBase;

    %Dead Reckoning — Updating Orientation
    theta_odom_new = theta_odom_previous + deltatheta;

    % LOCAL BODY FRAME velocities
    v_body_x = distance / dt; % Forward velocity
    v_body_y = 0;             % Ideal lateral slip is 0

    %Adding noise to the vx and vy
    v_body_x_noisy = v_body_x + noise_odo_vx*randn();
    v_body_y_noisy = v_body_y + noise_odo_vy*randn();

    % Project noisy local velocities into the GLOBAL frame
    vx_global = v_body_x_noisy * cos(theta_odom_new) - v_body_y_noisy * sin(theta_odom_new);
    vy_global = v_body_x_noisy * sin(theta_odom_new) + v_body_y_noisy * cos(theta_odom_new);

    %Calculating the noisy x and noisy y by integration
    x_odom_new = x_odom_previous + vx_global*dt;
    y_odom_new = y_odom_previous + vy_global*dt;

    %Putting the new values in the previous variables to be ready for the
    %next iteration
    previousrightEncoder = newrightEncoder;
    previousleftEncoder = newleftEncoder;
    x_odom_previous = x_odom_new;
    y_odom_previous = y_odom_new;
    theta_odom_previous = theta_odom_new;

    %Printing the output
    fprintf('x_gps=%.2f y_gps=%.2f yaw=%.2f x_odom=%.2f y_odom=%.2f\n', ...
        gpsX, gpsY, yaw, x_odom_new, y_odom_new);
    
    % Example command for the robot to move
    sim.setJointTargetVelocity(leftMotor, 1.0);
    sim.setJointTargetVelocity(rightMotor, 2.0);

    pause(dt);
end

%Those three lines actually do not run if our simulationiteration is inf
sim.setJointTargetVelocity(leftMotor, 0);
sim.setJointTargetVelocity(rightMotor, 0);

sim.stopSimulation();
