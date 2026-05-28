function sensors = readSensors(gt,raw ,parameters) % to be updated

%gt = ground truth from coppelia sim
%raw = captured by datalogger

rng(0)% to be checked later (used as exèlained by estimation course professor)

N = numel(gt.t);

% for the accelerometer 
accelB = raw.accel;
% we flip the sign because in the sensor it is -9.81
accelB(:,3)=-accelB(:,3);


% add gaussian noise with a zero mean value
accelB(:,1) = accelB(:,1) + parameters.noise_accel * randn(N,1);
accelB(:,2) = accelB(:,2) +parameters.noise_accel * randn(N,1);
accelB(:,3) = accelB(:,3)  + parameters.noise_accel * randn(N,1);

%
gyroB= raw.gyro;  % bas 3mlna copy to avaod changing raw directly


% add noise
gyroB(:, 1) = gyroB(:, 1) + parameters.noise_gyro * randn(N, 1);
gyroB(:, 2) = gyroB(:, 2) + parameters.noise_gyro * randn(N, 1);
gyroB(:, 3) = gyroB(:, 3) + parameters.noise_gyro * randn(N, 1);
 

% 
r= parameters.wheelRadius;
vx_odo =r*(gt.wheelOmega(:,2)+ gt.wheelOmega(:,1))/2;
vy_odo= zeros(N,1); % from professors slides for differential drive robots
odoVelB = [ vx_odo + parameters.noise_odo_x * randn(N, 1), vy_odo + parameters.noise_odo_y * randn(N, 1) ];
 

 % NOTE: gps already in 
gpsW = raw.gps(:,1:2) + parameters.noise_gps * randn(N, 2); 
sensors.t       = gt.t;
sensors.accelB  = accelB;
sensors.gyroB   = gyroB;
sensors.odoVelB = odoVelB;
sensors.gpsW    = gpsW;
 
end