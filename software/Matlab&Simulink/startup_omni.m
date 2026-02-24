% startup_omni.m
clear; clc;
project_root = fileparts(mfilename('fullpath'));
addpath(genpath(project_root));
disp('All folders added to MATLAB path.');
run('config_hardware.m');
disp('Hardware configuration loaded.');
