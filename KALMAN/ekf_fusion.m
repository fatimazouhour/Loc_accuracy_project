clear all
clc


Base = load('kf_input.mat');
KF1  = load('kf1_output.mat');
KF2  = load('kf2_output.mat');

% Extract Data
truth_x = Base.true_x_hist;
truth_y = Base.true_y_hist;

x_ekf1 = KF1.kf1.pos(1, :);
y_ekf1 = KF1.kf1.pos(2, :);

x_ekf2 = KF2.kf2.pos(1, :);
y_ekf2 = KF2.kf2.pos(2, :);

%% 2. Complementary Filter (Eq. 2 from the paper)
% Set the blending parameters (must sum to 1)
% Tune alpha_1 based on which system has less noise in your specific run
alpha_1 = 0.25;           % Weight for EKF-1 (INS)
alpha_2 = 1 - alpha_1;   % Weight for EKF-2 (Odometer)

% Apply the fusion equation
x_fused = (alpha_1 .* x_ekf1) + (alpha_2 .* x_ekf2);
y_fused = (alpha_1 .* y_ekf1) + (alpha_2 .* y_ekf2);

%% 3. RMSE Calculation
rmse = @(est, truth) sqrt(mean((truth - est).^2));

r_ekf1_x = rmse(x_ekf1, truth_x);
r_ekf1_y = rmse(y_ekf1, truth_y);

r_ekf2_x = rmse(x_ekf2, truth_x);
r_ekf2_y = rmse(y_ekf2, truth_y);

r_fused_x = rmse(x_fused, truth_x);
r_fused_y = rmse(y_fused, truth_y);

fprintf('\n=== RMSE (meters) ===\n');
fprintf('                 x         y\n');
fprintf('EKF-1 (INS):   %7.4f   %7.4f\n', r_ekf1_x, r_ekf1_y);
fprintf('EKF-2 (Odo):   %7.4f   %7.4f\n', r_ekf2_x, r_ekf2_y);
fprintf('Fused Output:  %7.4f   %7.4f\n', r_fused_x, r_fused_y);

%% 4. Plotting the Fusion Results
figure('Name', 'Complementary Fusion: EKF1 + EKF2', 'NumberTitle', 'off', 'Position', [150, 150, 900, 700]);
hold on; grid on; axis equal;

% Ground Truth
plot(truth_x, truth_y, 'g-', 'LineWidth', 2.5, 'DisplayName', 'Ground Truth');

% EKF-1 Output
plot(x_ekf1, y_ekf1, '-', 'Color', [0.850 0.325 0.098], 'LineWidth', 1.2, 'DisplayName', 'EKF-1 (INS)');

% EKF-2 Output
plot(x_ekf2, y_ekf2, 'b-', 'LineWidth', 1.2, 'DisplayName', 'EKF-2 (Odo)');

% Fused Output
plot(x_fused, y_fused, 'k--', 'LineWidth', 2, 'DisplayName', sprintf('Fused (\\alpha_1=%.2f)', alpha_1));

xlabel('East Position X (meters)', 'FontWeight', 'bold');
ylabel('North Position Y (meters)', 'FontWeight', 'bold');
title('Localization Fusion: Complementary Filter', 'FontSize', 12);
legend('show', 'Location', 'best');

% Mark Start Point
plot(truth_x(1), truth_y(1), 'k^', 'MarkerSize', 8, 'MarkerFaceColor', 'y', 'HandleVisibility', 'off');
text(truth_x(1)+0.2, truth_y(1), 'Start', 'FontWeight', 'bold');

hold off;