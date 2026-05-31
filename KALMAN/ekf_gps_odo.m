close all
clc

addpath('C:\Users\Fatima\Desktop\Mechatronics\Robotics\Robotics_Project\Base');
S = load('kf_input.mat');


time  = S.time_hist(:).';
gps   = [S.gps_x_hist(:).'; S.gps_y_hist(:).'];
truth = [S.true_x_hist(:).'; S.true_y_hist(:).'];
N     = numel(time);

if N > 1 
    dt = median(diff(time));
else
    dt = 0.05; % Default CoppeliaSim dt
end


v_in = S.odom_v_hist(:).';   % noisy forward body speed  [m/s]
w_in = S.odom_w_hist(:).';   % wheel-derived yaw rate    [rad/s] = deltatheta/dt

%% same as ekf 1 ! recheck !
if isfield(S, 'initial_heading')
    theta0  = S.initial_heading;  var_th0 = (0.1)^2;
else
    theta0  = 0;                  var_th0 = (pi/2)^2;
    warning('initial_heading not in kf_input.mat; theta0=0 (large variance).');
end

%% tuned parameters
% we started with initial values given
sigma_gps = 0.3162;
sigma_v   = 0.3317;
sigma_w   = 0.05;      % <-- tune this first if filter diverges

R  = diag([sigma_gps^2, sigma_gps^2]);   % measurement noise  (GPS positions)
Qu = diag([sigma_v^2,   sigma_w^2  ]);   % control-input noise ([v, omega])

%% dimensions
n  = 3;                    % state: [x, y, theta]
H  = [1 0 0; 0 1 0];      % GPS selects x and y from the state

%% gps artificial outages
gps_available = true(1, N);
% Uncomment to test a GPS outage (Stage 5):
% gps_available(round(0.4*N) : round(0.4*N)+200) = false;

%% initializations (nfs ekf) 
x      = zeros(n, N);      % posterior (updated) state
P      = zeros(n, n, N);   % posterior covariance
x_pred = zeros(n, N);      % prior (predicted) state — kept for diagnostics

% Initialise with first GPS fix + initial heading
x(:,1)   = [gps(1,1); gps(2,1); theta0];
P(:,:,1) = diag([sigma_gps^2, sigma_gps^2, var_th0]);
x_pred(:,1) = x(:,1);

%% ekf loop 
I3 = eye(n);

for i = 2:N
    prev = x(:, i-1); 
    th = prev(3);
    v = v_in(i);  
    om = w_in(i);

    xp = [prev(1) + v*cos(th)*dt;prev(2) + v*sin(th)*dt;prev(3) + om*dt];
    xp(3) = atan2(sin(xp(3)), cos(xp(3)));   % wrap to (-pi, pi)
    x_pred(:, i) = xp;

    % Jacobian of f w.r.t. state
    F = [1,  0,  -v*sin(th)*dt;
         0,  1,   v*cos(th)*dt;
         0,  0,   1           ];

    % Jacobian of f w.r.t. control inputs [v, omega]  (G = df/du)
    %   df_x / dv = cos(th) dt    df_x / domega = 0
    %   df_y / dv = sin(th) dt    df_y / domega = 0
    %   dtheta/dv = 0             dtheta/domega = dt
    G = [cos(th)*dt,  0;       
         sin(th)*dt,  0;
         0,           dt];

    Pp = F*P(:,:,i-1)*F' + G*Qu*G' + 1e-9*I3;   % prior covariance

    %filter form 
    if gps_available(i)
        Sinn = H*Pp*H' + R;              % innovation covariance
        K    = (Pp*H') / Sinn;           % Kalman gain  (3x2)
        xp   = xp + K*(gps(:,i) - H*xp);
        IKH  = I3 - K*H;
        Pp   = IKH*Pp*IKH' + K*R*K';    % Joseph form — numerically stable
    end

    xp(3) = atan2(sin(xp(3)), cos(xp(3)));   % wrap after update too
    x(:,i)   = xp;
    P(:,:,i) = (Pp + Pp') / 2;               % enforce symmetry
end

%% packaging to use 
velw = [v_in .* cos(x(3,:));
        v_in .* sin(x(3,:))];

kf2 = struct('t',time, 'pos',x(1:2,:), 'vel',  velw,'theta',x(3,:),'P', P,'Ppos', P(1:2,1:2,:),'gps_available', gps_available,'truth',truth);
save('kf2_output.mat', 'kf2');

%% smoothed gps ( nfs ekf1)
win    = 50;
gps_sm = [smoothdata(gps(1,:), 'sgolay', win);
          smoothdata(gps(2,:), 'sgolay', win)];

%% rmse nfs taree2et ekf1
rmse   = @(est) sqrt(mean((truth - est).^2, 2));
r_ekf2 = rmse(x(1:2,:));
r_sm   = rmse(gps_sm);
r_raw  = rmse(gps);

fprintf('\nRMSE (m)           x         y\n');
fprintf('EKF-2 filter     %7.4f   %7.4f\n', r_ekf2(1), r_ekf2(2));
fprintf('Smoothed GPS     %7.4f   %7.4f\n', r_sm(1),   r_sm(2));
fprintf('Raw GPS          %7.4f   %7.4f\n', r_raw(1),  r_raw(2));

%% plots
figure; hold on; grid on; axis equal

plot(gps(1,:), gps(2,:), '.', 'Color', [1 .6 .6], 'MarkerSize', 4)
plot(gps_sm(1,:), gps_sm(2,:), '-', 'Color', [.2 .2 .2], 'LineWidth', 1.2)
plot(truth(1,:), truth(2,:), 'g-', 'LineWidth', 1.8)
plot(x(1,:), x(2,:), 'b-', 'LineWidth', 1.2)
plot(S.odom_x_hist, S.odom_y_hist, 'm--', 'LineWidth', 1.5) % Raw Odometry

legend('Raw GPS', 'Smoothed GPS', 'Truth', 'EKF-2', 'Raw Odometry', 'Location', 'best')
title('EKF-2: GPS + Odometer')
xlabel('x (m)'); ylabel('y (m)')

