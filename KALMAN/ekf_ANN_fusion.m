clear 
clc

addpath('C:\Users\Fatima\Desktop\Mechatronics\Robotics\Robotics_Project\KALMAN');
addpath('C:\Users\Fatima\Desktop\Mechatronics\Robotics\Robotics_Project\ANN');
%load data
Base = load('kf_input.mat');
K2   = load('kf2_output.mat');  kf2 = K2.kf2;

t     = Base.time_hist(:).';
N     = numel(t);
dt    = median(diff(t));
truth = [Base.true_x_hist(:).'; Base.true_y_hist(:).'];

gps_available = kf2.gps_available;        % reuse the EXACT EKF mask
if all(gps_available)
    warning(['gps_available is all true -> no outage, ANN never takes over. ' ...
             'Enable an outage in the EKF scripts so the mask propagates here.']);
end

%% ann inputs 

accel   = [Base.accelX_hist(:), Base.accelY_hist(:), zeros(N,1)]; % [ax ay az]
velINS  = kf2.vel.';                       % KF-2 world velocity [vx vy]
heading = kf2.theta(:);                    % KF-2 heading [rad]
odoVel  = [Base.odom_v_hist(:), zeros(N,1)];   % odometer body velocity [vx vy]

% ANN feature matrix eq 3 in paper

acc_mag     = sqrt(sum(accel.^2, 2));
cum_acc_mag = cumsum(acc_mag) * dt;
vel_mag     = sqrt(sum(velINS.^2, 2));
cum_vel_mag = cumsum(vel_mag) * dt;
dpsi        = [0; diff(unwrap(heading))];  % per-step heading change
cum_heading = cumsum(dpsi);                % total accumulated turn [rad]
vx_odo      = odoVel(:,1);   cum_vx_odo = cumsum(vx_odo) * dt;
vy_odo      = odoVel(:,2);   cum_vy_odo = cumsum(vy_odo) * dt;

X = [acc_mag, cum_acc_mag, vel_mag, cum_vel_mag, heading, ...
     cum_heading, vx_odo, cum_vx_odo, vy_odo, cum_vy_odo];   % N x 10

%wanted position
Y = kf2.pos.';                             % N x 2

%train ann on gps data only 
tr_idx = find(gps_available);
if numel(tr_idx) < 20
    error('Too few GPS-available samples to train the ANN.');
end
ann = RobotANN_Class(size(X,2));
ann.trainANN(X(tr_idx,:), Y(tr_idx,:));

%ann prediction
pos_ann = ann.predict(X).';                % 2 x N

%%fusion ann+kf2 see paper
a1_fuzzy = 0.5;                            % <-- FLS HOOK (fixed for now)
final = zeros(2,N);
for k = 1:N
    if gps_available(k)
        final(:,k) = kf2.pos(:,k);         % GPS up: KF-2 is the estimate
    else
        final(:,k) = a1_fuzzy*pos_ann(:,k) + (1-a1_fuzzy)*kf2.pos(:,k);  % Eq.5
    end
end

%RMSE
rmse = @(e) sqrt(mean(sum(e.^2,1)));
out  = ~gps_available;
fprintf('\n=== Combined position RMSE (m) ===\n');
if any(out)
    fprintf('-- during GPS OUTAGE --\n');
    fprintf('KF-2 alone : %.4f\n', rmse(truth(:,out)-kf2.pos(:,out)));
    fprintf('ANN alone  : %.4f\n', rmse(truth(:,out)-pos_ann(:,out)));
    fprintf('Fusion B   : %.4f   (a1_fuzzy=%.2f)\n', rmse(truth(:,out)-final(:,out)), a1_fuzzy);
end
fprintf('-- whole run --\n');
fprintf('KF-2     : %.4f\n', rmse(truth-kf2.pos));
fprintf('Fusion B : %.4f\n', rmse(truth-final));

%% plots
figure; hold on; grid on; axis equal
plot(truth(1,:),  truth(2,:),  'g-',  'LineWidth',2,   'DisplayName','Truth')
plot(kf2.pos(1,:),kf2.pos(2,:),'b-',  'LineWidth',1,   'DisplayName','KF-2')
plot(pos_ann(1,:),pos_ann(2,:),'-',   'Color',[.85 .33 .1],'DisplayName','ANN')
plot(final(1,:),  final(2,:),  'k--', 'LineWidth',1.8, 'DisplayName','Fusion B (ANN+KF-2)')
if any(out)
    plot(truth(1,out),truth(2,out),'rx','MarkerSize',5,'DisplayName','outage truth')
end
legend('Location','best'); xlabel('x (m)'); ylabel('y (m)')
title('Fusion B (paper Eq. 5): ANN + KF-2 during GPS outage')