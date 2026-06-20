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
