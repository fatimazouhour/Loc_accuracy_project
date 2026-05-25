clc;
clear;

addpath('/home/husain5/Documents/CoppeliaSim/CoppeliaSim_Edu_V4_10_0_rev0_Ubuntu22_04/programming/zmqRemoteApi/clients/matlab');
rehash toolboxcache;

client = RemoteAPIClient();
sim = client.require('sim');

robot = sim.getObject('/PioneerP3DX');
leftMotor = sim.getObject('/PioneerP3DX/leftMotor');
rightMotor = sim.getObject('/PioneerP3DX/rightMotor');
simulationiteration = 50
sim.startSimulation();
noise_accel= 0.003; % consistent with Mpu-9150 data sheet mentioned in the paper
noise_gyro= 0.005; % ...  
noise_odo_x = 0.2;   % the paper reflects the REAL pioneer 
noise_odo_y= 0.02;
noise_gps = 0.3161; %consistent with  uBlox NEO-6m datasheet

%This for loops runs infinitly until we stop the matlab execution manually
%to stop printing data and from the simulation to make the robot stop
%running
while sim.getSimulationTime() < simulationiteration

    % Getting the Position using GPS from CoppeliaSim
    gpsX = sim.getFloatSignal('gpsX') + noise_gps*randn();
    gpsY = sim.getFloatSignal('gpsY') + noise_gps*randn();
    gpsZ = sim.getFloatSignal('gpsZ') + noise_gps*randn();
    
    gps = [gpsX gpsY gpsZ];

    %Getting the angles using Gyroscope from CoppeliaSim 
    roll = sim.getFloatSignal('roll') + noise_gyro*randn();
    pitch = sim.getFloatSignal('pitch') + noise_gyro*randn();
    yaw = sim.getFloatSignal('yaw') + noise_gyro*randn();

    angle = [roll,pitch,yaw];

    % Getting the acceleration using the Accelerometer from CoppeliaSim
    accelX = sim.getFloatSignal('accelX') + noise_accel*randn();
    accelY = sim.getFloatSignal('accelY') + noise_accel*randn();
    accelZ = sim.getFloatSignal('accelZ') + noise_accel*randn();

    % Getting the left and right encoder using leftmotor and rightmore from Coppeliasim 
    leftEncoder = sim.getJointPosition(leftMotor);
    rightEncoder = sim.getJointPosition(rightMotor);

    fprintf('x=%.2f y=%.2f yaw=%.2f leftEnc=%.2f rightEnc=%.2f\n', ...
        gpsX, gpsY, yaw, leftEncoder, rightEncoder);
    
    % Example command for the robot to move
    sim.setJointTargetVelocity(leftMotor, 1.0);
    sim.setJointTargetVelocity(rightMotor, 2.0);

    pause(0.05);
end

%Those three lines actually do not run if our simulationiteration is inf
sim.setJointTargetVelocity(leftMotor, 0);
sim.setJointTargetVelocity(rightMotor, 0);

sim.stopSimulation();
