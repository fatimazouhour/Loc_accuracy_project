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
% run('ekf_ANN_fusion'); 

disp('finished.');