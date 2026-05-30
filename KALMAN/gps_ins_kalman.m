clear all
close all
clc

% for simplicity , we decided to follow  our estimation course ekf structure
addpath('C:\Users\Fatima\Desktop\Mechatronics\Robotics\Robotics_Project\Base');
load('kf_input.mat');

N=numel(time_hist); % nb of data given
dt=0.05;
N0_vector = [0,100,600]; % TO BE CHECKED randomly chosen
% definition of lti synamic system

A = [1 0 dt 0;0 1 0 dt;0 0 1  0;0 0 0  1];
B = [0.5*dt^2 0; 0 0.5*dt^2;dt  0; 0 dt];
C = [1 0 0 0;0 1 0 0];
D = zeros(2,2);

[n,nn] = size(A);
q = size(C,1);


noise_gps = 0.3162;  % GPS std [m] (paper value)
V1  = diag([1e-4, 1e-4, 1e-2, 1e-2]);     % tune: pos small, vel larger
V2  = diag([noise_gps^2, noise_gps^2]);
V12 = zeros(n,q); % covar matrix set to zero since technically ma by2asro 3 b3d 


yaw     = 0;
yawSign = -1;          % matches BaseMatlab theta - gyro_r*dt; flip if mirrored
u = zeros(2, N);
y = zeros(q, N);

% measuring y adn u from sensors
for t = 1:N
    yaw  = yaw + yawSign * gyro_r_hist(t) * dt;
    ax_b =  accelX_hist(t);
    ay_b = -accelY_hist(t);                        % BaseMatlab's -accel(2) flip
    u(1,t) = cos(yaw)*ax_b - sin(yaw)*ay_b;        % ax world
    u(2,t) = sin(yaw)*ax_b + cos(yaw)*ay_b;        % ay world
    y(:,t) = [gps_x_hist(t); gps_y_hist(t)];
end

% for comparison only 
x_true = [true_x_hist(:)'; true_y_hist(:)']; 

% inizialization
x_h(:,1) = [gps_x_hist(1); gps_y_hist(1); 0; 0];
P{1} = diag([noise_gps^2, noise_gps^2, 1, 1])


% KALMAN filter+predictor (one-step)

for t = 1:N
    y_h(:,t)   = C*x_h(:,t);                            % predicted output
    e(:,t)     = y(:,t) - y_h(:,t);                     % innovation
    K{t}       = (A*P{t}*C' + V12) / (C*P{t}*C' + V2);  % predictor gain
    x_h(:,t+1) = A*x_h(:,t) + B*u(:,t) + K{t}*e(:,t);   % next prediction
    P{t+1}     = A*P{t}*A' + V1 - K{t}*(C*P{t}*C' + V2)*K{t}';
 
    K0{t}    = P{t}*C' / (C*P{t}*C' + V2);              % filter gain
    x_f(:,t) = x_h(:,t) + K0{t}*e(:,t);                 % filtered estimate
    y_f(:,t) = C*x_f(:,t);
end

% to check how 'good' the filter is we use RMSE

for ind = 1:length(N0_vector)
    N0 = N0_vector(ind);
    for k = 1:2
        RMSE_x_h(k,ind) = norm(x_true(k,N0+1:N) - x_h(k,N0+1:N))/sqrt(N-N0);
        RMSE_x_f(k,ind) = norm(x_true(k,N0+1:N) - x_f(k,N0+1:N))/sqrt(N-N0);
    end
    RMSE_gps(1,ind) = norm(x_true(1,N0+1:N) - gps_x_hist(N0+1:N))/sqrt(N-N0);
    RMSE_gps(2,ind) = norm(x_true(2,N0+1:N) - gps_y_hist(N0+1:N))/sqrt(N-N0);
end
fprintf('\n           N0=%d     N0=%d    N0=%d\n', N0_vector(1),N0_vector(2),N0_vector(3));
disp('RMSE_x_h  (predictor K) [x;y]:'), disp(RMSE_x_h)
disp('RMSE_x_f  (filter F)    [x;y]:'), disp(RMSE_x_f)
disp('RMSE_gps  (raw GPS)     [x;y]:'), disp(RMSE_gps)


figure, hold on, grid on, axis equal
plot(gps_x_hist, gps_y_hist, 'r.', 'MarkerSize',4)
plot(x_true(1,1:N), x_true(2,1:N), 'g-',  'LineWidth',1.5)
plot(x_f(1,1:N),    x_f(2,1:N),    'b--', 'LineWidth',1.5)
plot(x_h(1,1:N),    x_h(2,1:N),    'm-.', 'LineWidth',1.0)

xlabel('x (m)'), ylabel('y (m)'), title('GPS-INS: trajectory')
legend('GPS (meas)','System S (truth)','Filter F','Predictor K')
 

x_h_ss(:,1) = [gps_x_hist(1); gps_y_hist(1); 0; 0];
Sys1 = ss(A, [B, eye(n)], C, [D, zeros(q,n)], dt);
[~, Kbar, ~, K0bar] = kalman(Sys1, V1, V2, 0);


