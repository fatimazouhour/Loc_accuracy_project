clear; clc; close all;

here = fileparts(mfilename('fullpath'));
if isempty(here), here = pwd; end   

addpath(fullfile(here, 'Base'));     
addpath(fullfile(here, 'KALMAN'));   
addpath(fullfile(here, 'ANN'));      

cd(here);

run('BaseMatlab');     
run('ekf_gps_ins');      
run('ekf_gps_odo');      
run('ekf_fusion'); 

FORCE_RETRAIN = false; 

if FORCE_RETRAIN || ~exist('trainedANN.mat', 'file')
    fprintf('Training model because it is missing or forced...\n');
    % This calls your new training/validation file
    run('ann_training'); 
else
    fprintf('Loading existing trained model...\n');
    % This loads the model object directly into your workspace
    load('trainedANN.mat', 'ann');
end

run('ekf_ann_fusion'); 

disp('finished.');

%% =========================
%  LOAD RESULTS 

load('results.mat')
kf2     = results.kf2;
final   = results.final;
pos_ann = results.pos_ann;
truth   = results.truth;
gps     = results.gps;

N = size(truth,2);
t = 1:N;

pos_ann(isnan(pos_ann)) = 0;

kf_x = kf2.pos(1,:);
kf_y = kf2.pos(2,:);

truth_x = truth(1,:);
truth_y = truth(2,:);

fus_x = final(1,:);
fus_y = final(2,:);

ann_x = pos_ann(1,:);
ann_y = pos_ann(2,:);
out = find(~gps);

load('results.mat','results');

truth_x = results.truth(1,:);
truth_y = results.truth(2,:);

kf_x = results.kf2.pos(1,:);
kf_y = results.kf2.pos(2,:);

fus_x = results.final(1,:);
fus_y = results.final(2,:);

ann_x = results.pos_ann(1,:);
ann_y = results.pos_ann(2,:);

t = results.time(:)';
gps = results.gps(:)';
out = find(~gps);
N = length(t);


% 1. MAIN TRAJECTORY PLOT

figure('Color','w');
hold on;
grid on;
axis equal;

plot(truth_x, truth_y, 'g-', 'LineWidth', 2);
plot(kf_x, kf_y, 'b-', 'LineWidth', 1.5);
plot(fus_x, fus_y, 'k--', 'LineWidth', 2);

if ~isempty(out)
    plot(truth_x(out), truth_y(out), ...
        'rx', 'MarkerSize', 5, 'LineWidth', 1.5);
end

xlabel('X (m)');
ylabel('Y (m)');
title('Robot Localization Results');

legend('Truth','KF-2','Fusion','Outage');

% 2. OUTAGE SEGMENTS ONLY

if ~isempty(out)

    d = diff(out);
    starts = [1; find(d > 1) + 1];
    ends   = [starts(2:end)-1; length(out)];

    figure('Color','w');
    hold on;
    grid on;
    axis equal;

    for k = 1:length(starts)

        idx = out(starts(k):ends(k));

        plot(truth_x(idx), truth_y(idx),'g', 'LineWidth', 2);

        plot(kf_x(idx), kf_y(idx),'b', 'LineWidth', 1.5);

        plot(ann_x(idx), ann_y(idx),'r', 'LineWidth', 1.5);

        plot(fus_x(idx), fus_y(idx), 'k', 'LineWidth', 2);

    end

    xlabel('X (m)');
    ylabel('Y (m)');
    title('GPS Outage Regions');

    legend('Truth','KF','ANN','Fusion');

end


% 3. X ERROR OVER TIME

kf_err_x  = kf_x  - truth_x;
ann_err_x = ann_x - truth_x;
fus_err_x = fus_x - truth_x;

figure('Color','w');
hold on;
grid on;

plot(t, kf_err_x,'b', 'LineWidth', 1.5);

plot(t, ann_err_x,'r', 'LineWidth', 1.5);

plot(t, fus_err_x,'k', 'LineWidth', 1.5);

xlabel('Time (s)');
ylabel('X Error (m)');
title('X Position Error');

legend('KF-2','ANN','Fusion');


% 4. Y ERROR OVER TIME


kf_err_y  = kf_y  - truth_y;
ann_err_y = ann_y - truth_y;
fus_err_y = fus_y - truth_y;

figure('Color','w');
hold on;
grid on;

plot(t, kf_err_y,'b', 'LineWidth', 1.5);

plot(t, ann_err_y,'r', 'LineWidth', 1.5);

plot(t, fus_err_y,'k', 'LineWidth', 1.5);

xlabel('Time (s)');
ylabel('Y Error (m)');
title('Y Position Error');

legend('KF-2','ANN','Fusion');
