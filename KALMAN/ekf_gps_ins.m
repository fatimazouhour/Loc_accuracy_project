clear all
close all
clc

use_nonholonomic = true;     % fuse zero-lateral-velocity constraint (paper)   

S = load('kf_input.mat');
time   = S.time_hist(:).';
gps = [S.gps_x_hist(:).';  S.gps_y_hist(:).'];
acc  = [S.accelX_hist(:).'; S.accelY_hist(:).'];
gyro_r = S.gyro_r_hist(:).';
truth = [S.true_x_hist(:).'; S.true_y_hist(:).'];
N  = numel(time);
if N > 1 
    dt = median(diff(time));
else
    dt = 0.05;
end
 
%% Recheck !!!
if isfield(S,'initial_heading')
    theta0 = S.initial_heading;  var_th0 = (0.1)^2;
else
    theta0 = 0;                  var_th0 = (pi/2)^2;
    warning('initial_heading not in kf_input.mat; theta0=0 (large variance).');
end
yawSign = -1;                    % BaseMatlab INS: theta = theta - gyro_r*dt
 
%% to be tuned
sigma_gps    = 0.3162;   % GPS std [m]
sigma_a_proc = 0.8;      % unmodeled accel uncertainty [m/s^2]  (main knob)
sigma_w_proc = 0.5;      % unmodeled yaw-rate uncertainty [rad/s]
R  = diag([sigma_gps^2, sigma_gps^2]);
Qu = diag([sigma_a_proc^2, sigma_a_proc^2, sigma_w_proc^2]);

n = 5;
H = [1 0 0 0 0; 0 1 0 0 0];
u = [acc(1,:); -acc(2,:); gyro_r];
 
%% gps artificial outage
gps_available = true(1,N);
%gps_available(round(0.4*N):round(0.4*N)+200) = false;   % example outage
 
%% ekf loop 
I = eye(n);
x = zeros(n,N);
P = zeros(n,n,N);
x(:,1)   = [gps(1,1); gps(2,1); 0; 0; theta0];
P(:,:,1) = diag([sigma_gps^2, sigma_gps^2, 1, 1, var_th0]);
 
for i = 2:N
    prev = x(:,i-1);  th = prev(5);
    ax_b = u(1,i); ay_b = u(2,i); r = u(3,i);
    aw_x = cos(th)*ax_b - sin(th)*ay_b;
    aw_y = sin(th)*ax_b + cos(th)*ay_b;
 
    % predict
    xp = [ prev(1)+prev(3)*dt+0.5*aw_x*dt^2;
           prev(2)+prev(4)*dt+0.5*aw_y*dt^2;
           prev(3)+aw_x*dt;
           prev(4)+aw_y*dt;
           prev(5)+yawSign*r*dt ];
    xp(5) = atan2(sin(xp(5)),cos(xp(5)));
    F = [1 0 dt 0 -0.5*dt^2*aw_y;
         0 1 0 dt  0.5*dt^2*aw_x;
         0 0 1 0  -dt*aw_y;
         0 0 0 1   dt*aw_x;
         0 0 0 0   1];
    G = [0.5*dt^2*cos(th) -0.5*dt^2*sin(th) 0;
         0.5*dt^2*sin(th)  0.5*dt^2*cos(th) 0;
         dt*cos(th)       -dt*sin(th)       0;
         dt*sin(th)        dt*cos(th)       0;
         0        0  yawSign*dt];
    Pp = F*P(:,:,i-1)*F' + G*Qu*G' + 1e-9*I;
 
    % update (GPS), Joseph form
    if gps_available(i)
        K  = (Pp*H')/(H*Pp*H' + R);
        xp = xp + K*(gps(:,i) - H*xp);
        IKH = I - K*H;
        Pp  = IKH*Pp*IKH' + K*R*K';
    end
    xp(5) = atan2(sin(xp(5)),cos(xp(5)));
    x(:,i)   = xp;
    P(:,:,i) = (Pp+Pp')/2;
end
 
%% from base matlab smoothed gps version
win = 50;
gps_sm = [smoothdata(gps(1,:),'sgolay',win);smoothdata(gps(2,:),'sgolay',win)];
 
%% output to use in fusion +ann
kf1 = struct('t',time,'pos',x(1:2,:),'vel',x(3:4,:),'theta',x(5,:), ...
             'P',P,'Ppos',P(1:2,1:2,:),'gps_available',gps_available,'truth',truth);
save('kf1_output.mat','kf1');
 
%% rmse
rmse  = @(est) sqrt(mean((truth - est).^2, 2));
r_ekf = rmse(x(1:2,:));
r_sm  = rmse(gps_sm);
r_raw = rmse(gps);
fprintf('\nRMSE (m)         x         y\n');
fprintf('EKF-1 filter   %7.4f   %7.4f\n', r_ekf(1), r_ekf(2));
fprintf('Smoothed GPS   %7.4f   %7.4f\n', r_sm(1),  r_sm(2));
fprintf('Raw GPS        %7.4f   %7.4f\n', r_raw(1), r_raw(2));
 
%% plots
figure; hold on; grid on; axis equal
plot(gps(1,:),    gps(2,:),    '.', 'Color',[1 .6 .6], 'MarkerSize',4)
plot(gps_sm(1,:), gps_sm(2,:), '-', 'Color',[.2 .2 .2], 'LineWidth',1.2)
plot(truth(1,:),  truth(2,:),  'g-', 'LineWidth',1.8)
plot(x(1,:),      x(2,:),      'b-', 'LineWidth',1.2)
legend('Raw GPS','Smoothed GPS','Truth','EKF-1','Location','best')
title('EKF-1: GPS-INS vs smoothed GPS'); xlabel('x (m)'); ylabel('y (m)')