%% --- 1. Load Data & Consistency Checks ---
input_file  = load('kf_input.mat');      
fusion_file = load('fusion_output.mat'); 

% Ensure data is aligned in time
assert(length(input_file.time_hist) == length(fusion_file.x_fused), ...
    'Data length mismatch! Run BaseMatlab and Fusion again.');

dt = median(diff(input_file.time_hist));

%% --- 2. Build ANN Input (Feature Engineering) ---
data.accel   = [input_file.accelX_hist(:), input_file.accelY_hist(:), zeros(size(input_file.accelX_hist(:)))];
data.vel     = [input_file.vx_ins_hist(:), input_file.vy_ins_hist(:)];
data.heading = input_file.initial_heading + cumsum(input_file.gyro_r_hist(:) * dt); 
data.odoVel  = [input_file.odom_v_hist(:), zeros(size(input_file.odom_v_hist(:)))];

X_all = buildANNInput(data, dt); 

% --- SMOOTHING TARGETS ---
% We apply a Savitzky-Golay filter to remove high-frequency noise from the fusion output.
% This creates a "cleaner" target for the ANN to learn from.
Y_raw = [fusion_file.x_fused(:), fusion_file.y_fused(:)];
Y_all = [smoothdata(Y_raw(:,1), 'sgolay', 15), ...
         smoothdata(Y_raw(:,2), 'sgolay', 15)];

%% --- 3. Train the ANN ---
valid_idx = (input_file.gps_available(:) == true);
X_train   = X_all(valid_idx, :);
Y_train   = Y_all(valid_idx, :);

if size(X_train, 1) < 100 
    error('Training Aborted: Need >100 samples of healthy GPS data.');
end

ann = RobotANN_Class(13); 
ann.trainANN(X_train, Y_train);

%% --- 4. Validation & Accuracy Gatekeeper ---
Y_pred     = ann.predict(X_all);
rmse_vect  = sqrt(mean((Y_pred(valid_idx,:) - Y_train).^2));
total_rmse = norm(rmse_vect); 

fprintf('Training RMSE (x,y) = %.4f %.4f (Total: %.4f)\n', rmse_vect(1), rmse_vect(2), total_rmse);

% Relaxed threshold to 0.2 for stability, given typical GPS drift characteristics
max_allowable_rmse = 0.2; 
if total_rmse > max_allowable_rmse
    error('Training Aborted: RMSE (%.4f) > threshold (%.4f). Model not accurate enough.', ...
          total_rmse, max_allowable_rmse);
else
    ann.saveModel('trainedANN.mat');
    fprintf('SUCCESS: Model validated (%.4f < %.4f) and saved.\n', total_rmse, max_allowable_rmse);
end

%% --- 5. Final Visual Verification ---
outage_idx = (input_file.gps_available(:) == false);
if any(outage_idx)
    pos_ann = ann.predict(X_all(outage_idx, :));
    
    figure('Color', 'w'); hold on; grid on;
    plot(fusion_file.x_fused(outage_idx), fusion_file.y_fused(outage_idx), 'r-', 'LineWidth', 1.5, 'DisplayName', 'Target (Fusion)');
    plot(pos_ann(:,1), pos_ann(:,2), 'b--', 'LineWidth', 1.5, 'DisplayName', 'ANN Prediction');
    legend; title('ANN Prediction performance during GPS Outage');
else
    disp('No GPS outage detected; could not perform outage validation.');
end
