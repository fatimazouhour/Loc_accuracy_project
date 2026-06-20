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

% run('ekf_ANN_fusion'); 

disp('finished.');
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
