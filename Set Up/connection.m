%% connection

vrep = remApi('remoteApi');
vrep.simxFinish(-1);

clientID = vrep.simxStart('127.0.0.1', 19999, true, true, 5000, 5);

%% connection check delete b3den

if (clientID > -1)
    disp('>>> Connected! <<<');
    
    % Use the full path to avoid Error 64
    [err1, left_motor] = vrep.simxGetObjectHandle(clientID, '/PioneerP3DX/leftMotor', vrep.simx_opmode_blocking);
    [err2, right_motor] = vrep.simxGetObjectHandle(clientID, '/PioneerP3DX/rightMotor', vrep.simx_opmode_blocking);
    
    if (err1 == 0 && err2 == 0)
        % Circular speeds
        vrep.simxSetJointTargetVelocity(clientID, left_motor, 1.5, vrep.simx_opmode_oneshot);
        vrep.simxSetJointTargetVelocity(clientID, right_motor, 4.0, vrep.simx_opmode_oneshot);
        
        pause(5); % Move for 5 seconds
        
        % Stop
        vrep.simxSetJointTargetVelocity(clientID, left_motor, 0, vrep.simx_opmode_oneshot);
        vrep.simxSetJointTargetVelocity(clientID, right_motor, 0, vrep.simx_opmode_oneshot);
        disp('Finished Circle.');
    else
        fprintf('Still missing handles. Error codes: L=%d, R=%d\n', err1, err2);
    end
    
    % Finish the connection ONLY at the end
    vrep.simxFinish(clientID);
else
    disp('Could not connect. Is CoppeliaSim PLAYING?');
end