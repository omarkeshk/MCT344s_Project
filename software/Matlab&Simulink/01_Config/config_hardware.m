% config_hardware.m
R = 0.0485;          % Wheel radius in meters
L = 0.20;          % Distance from center to wheel in meters
gear_ratio = 1;    % Motor gearbox ratio
max_rpm = 250;     % Max motor speed
max_rad_s = (max_rpm * 2 * pi) / 60; 

% Omni-wheel mounting angles
alpha_1 = pi/4;      % Front-Right
alpha_2 = 3*pi/4;    % Front-Left
alpha_3 = 5*pi/4;    % Back-Left
alpha_4 = 7*pi/4;    % Back-Right

alphas = [alpha_1, alpha_2, alpha_3, alpha_4];