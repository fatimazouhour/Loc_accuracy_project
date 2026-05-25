function sensors = readSensors(gt,raw ,parameters) % to be updated

%gt = ground truth from coppelia sim
%raw = captured by datalogger

randn('default') % to be checked later (used as exèlained by estimation course professor)

N = numel(gt.t);

% for the accelerometer 
accelB = raw.accel;
% we flip the sign because in the sensor it is -9.81
accelB(:,3)=-accel(:,3);


% add gaussian noise with a zero mean value
accelB(:,1) = accelB(:,1) + parameters.noise_accel * randn(N,1);
accelB(:,2) = accelB(:,2) +p.noise_accel * randn(N,1);
accelB(:,3) = accelB(:,3)  + p.noise_accel * randn(N,1);
 
gyroB        = gyroStream;
gyroB(:,3)   = gyroB(:,3) + p.bias.gyroZ + p.noise.gyro * randn(N,1);
gyroB(:,1:2) = gyroB(:,1:2)              + p.noise.gyro * randn(N,2);
 

r  = p.robot.wheelRadius;
vx = r * (gt.wheelOmega(:,2) + gt.wheelOmega(:,1)) / 2;
vy = zeros(N,1);
odoVelB = [vx + p.noise.odoVel*randn(N,1), ...
           vy + p.noise.odoVel*randn(N,1)];
 
gpsW = gt.posW(:,1:2) + p.noise.gpsPos * randn(N,2);
 
sensors.t       = gt.t;
sensors.accelB  = accelB;
sensors.gyroB   = gyroB;
sensors.odoVelB = odoVelB;
sensors.gpsW    = gpsW;
 
end