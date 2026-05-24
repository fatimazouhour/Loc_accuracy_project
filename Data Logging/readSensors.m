function sensors = readCoppeliaSensors(gt, accelStream, gyroStream, p) % to be updated
 
N = size(accelStream, 1);

 
accelB = accelStream;
accelB(:,1) = accelB(:,1) + parameters.noise_accel + parameters.noise.accel * randn(N,1);
accelB(:,2) = accelB(:,2) + parameters.bias_accel + p.noise.accel * randn(N,1);
accelB(:,3) = accelB(:,3)                + p.noise.accel * randn(N,1);
 
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