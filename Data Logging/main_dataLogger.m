clear % so that old variables from ùold sessions are not saved
close all
clc

addpath('..') 
config;
rng(0);

if ~exist(dataDir, 'dir')
    mkdir(dataDir);
end


%% connection to coppelia sim AGAIN

sim=remApi('remoteApi');
sim.simxFinish(-1);   % closing left overs from previous run 
clientID= sim.simxStart(ipAddress, port, true, true,5000,5);



if clientID < 0
    error('Could not connect to CoppeliaSim.'); % to visualize wether we established a connection or not 
end
disp('Connected to CoppeliaSim');


[rc1, robot]= sim.simxGetObjectHandle(clientID, '/PioneerP3DX', sim.simx_opmode_blocking);
[rc2, leftMotor]= sim.simxGetObjectHandle(clientID, '/PioneerP3DX/leftMotor', sim.simx_opmode_blocking);
[rc3, rightMotor]=sim.simxGetObjectHandle(clientID, '/PioneerP3DX/rightMotor', sim.simx_opmode_blocking);

%%

omega  = speed / radius;                  
vRight = speed + omega * wheelBase / 2;    
vLeft  = speed - omega * wheelBase / 2;   
wRight = vRight/wheelRadius;             
wLeft  = vLeft/wheelRadius;

% pos , vel, angles
sim.simxGetObjectPosition(clientID, robot, -1, sim.simx_opmode_streaming);
sim.simxGetObjectOrientation(clientID, robot, -1, sim.simx_opmode_streaming);
sim.simxGetObjectVelocity(clientID, robot, sim.simx_opmode_streaming);

%motors
sim.simxGetJointPosition(clientID, leftMotor, sim.simx_opmode_streaming);
sim.simxGetJointPosition(clientID, rightMotor, sim.simx_opmode_streaming);

% sensors
sim.simxGetStringSignal(clientID, 'accelData', sim.simx_opmode_streaming);
sim.simxGetStringSignal(clientID, 'gyroData', sim.simx_opmode_streaming);
sim.simxGetStringSignal(clientID, 'gpsData', sim.simx_opmode_streaming);
pause(0.3);

%% start trajectory
sim.simxSetJointTargetVelocity(clientID, leftMotor, wLeft, sim.simx_opmode_oneshot);
sim.simxSetJointTargetVelocity(clientID, rightMotor, wRight, sim.simx_opmode_oneshot);


% initiazlization 
N = round(T/dt);
gt.t= zeros(N,1);
gt.posW= zeros(N,3);
gt.eulW= zeros(N,3);
gt.linVelW= zeros(N,3);
gt.angVelW= zeros(N,3);
gt.wheelOmega= zeros(N,2);
raw.accel= zeros(N,3);
raw.gyro= zeros(N,3);
raw.gps= zeros(N,3);

prevLeft  = 0;
prevRight = 0;
firstRead = true;


%% main loop

% Main logging loop
for k = 1:N
    % Read ground truth from CoppeliaSim
    [~, pos]= sim.simxGetObjectPosition(clientID, robot, -1, sim.simx_opmode_buffer);
    [~, eul] = sim.simxGetObjectOrientation(clientID, robot, -1, sim.simx_opmode_buffer);
    [~, linV, angV] = sim.simxGetObjectVelocity(clientID, robot, sim.simx_opmode_buffer);
    [~, jL]= sim.simxGetJointPosition(clientID, leftMotor, sim.simx_opmode_buffer);
    [~, jR]= sim.simxGetJointPosition(clientID, rightMotor, sim.simx_opmode_buffer);

    % Compute wheel angular velocity by finite difference 
    if firstRead
        wL = 0;
        wR = 0;
        firstRead = false;
    else
        wL = wrapToPi(jL - prevLeft)  / dt;
        wR = wrapToPi(jR - prevRight) / dt;
    end
    prevLeft  = jL;
    prevRight = jR;

    % Read sensor signals from CoppeliaSim
    [~, aStr] = sim.simxGetStringSignal(clientID, 'accelData', sim.simx_opmode_buffer);
    [~, gStr] = sim.simxGetStringSignal(clientID, 'gyroData', sim.simx_opmode_buffer);
    [~, pStr] = sim.simxGetStringSignal(clientID, 'gpsData', sim.simx_opmode_buffer);

    a  = sim.simxUnpackFloats(aStr);
    gv = sim.simxUnpackFloats(gStr);
    pv = sim.simxUnpackFloats(pStr);

  
    gt.t(k)            = (k-1) * dt;
    gt.posW(k,:)       = pos(:)';
    gt.eulW(k,:)       = eul(:)';
    gt.linVelW(k,:)    = linV(:)';
    gt.angVelW(k,:)    = angV(:)';
    gt.wheelOmega(k,:) = [wL, wR];

    if numel(a)  >= 3, raw.accel(k,:) = a(1:3);  end
    if numel(gv) >= 3, raw.gyro(k,:)  = gv(1:3); end
    if numel(pv) >= 3, raw.gps(k,:)   = pv(1:3); end

    pause(dt);
end


%%
% stop motor 
sim.simxSetJointTargetVelocity(clientID, leftMotor, 0, sim.simx_opmode_oneshot);
sim.simxSetJointTargetVelocity(clientID, rightMotor, 0, sim.simx_opmode_oneshot);

% disconnect
sim.simxFinish(clientID);
sim.delete();
disp('Done logging');

% save everything 
sensors = readSensors(gt, raw, parameters);
save(fullfile(dataDir, dataFile), 'gt', 'sensors', 'parameters');
disp(['Saved to ' fullfile(dataDir, dataFile)]);

%plotting
figure;
plot(gt.posW(:,1), gt.posW(:,2), 'b', 'LineWidth', 1.5); hold on;
plot(sensors.gpsW(:,1), sensors.gpsW(:,2), 'r.', 'MarkerSize', 4);
axis equal; grid on;
xlabel('x (m)'); ylabel('y (m)');
legend('true path', 'GPS readings');
title('Robot trajectory');