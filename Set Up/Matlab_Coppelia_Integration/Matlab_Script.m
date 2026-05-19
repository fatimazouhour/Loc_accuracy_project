clc;
clear;

%adding the matlab file from zmqRemoteApi from the programming file in CoppeliaSim_Edu_V4_10_0_rev0_Ubuntu22_04
addpath('/home/husain5/Downloads/CoppeliaSim_Edu_V4_10_0_rev0_Ubuntu22_04/programming/zmqRemoteApi/clients/matlab');
rehash toolboxcache;

client = RemoteAPIClient();
sim = client.require('sim');

robot = sim.getObject('/PioneerP3DX');
leftMotor = sim.getObject('/PioneerP3DX/leftMotor');
rightMotor = sim.getObject('/PioneerP3DX/rightMotor');
simulationiteration = 1000
sim.startSimulation();

%This for loops runs infinitly until we stop the matlab execution manually
%to stop printing data and from the simulation to make the robot stop
%running
while sim.getSimulationTime() < simulationiteration

    % Getting the Position using GPS from CoppeliaSim
    gpsX = sim.getFloatSignal('gpsX');
    gpsY = sim.getFloatSignal('gpsY');
    gpsZ = sim.getFloatSignal('gpsZ');
    
    gps = [gpsX gpsY gpsZ];

    %Getting the angles using Gyroscope from CoppeliaSim 
    roll = sim.getFloatSignal('roll');
    pitch = sim.getFloatSignal('pitch');
    yaw = sim.getFloatSignal('yaw');

    angle = [roll,pitch,yaw];

    % Getting the acceleration using the Accelerometer from CoppeliaSim
    accelX = sim.getFloatSignal('accelX');
    accelY = sim.getFloatSignal('accelY');
    accelZ = sim.getFloatSignal('accelZ');

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
