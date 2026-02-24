% test_pure_y_forward.m
% Tests the straight-on forward motion for the RED Pickup/Drop-off zone

disp('--- Running Station A (Pure Y) Test ---');

% 1. Define target speeds: 0.5 m/s directly forward
target_Vx = 0.0; 
target_Vy = 0.5; 
target_Wz = 0.0;

% 2. Calculate raw wheel speeds (calls function from 02_Kinematics)
raw_speeds = inverse_kinematics(target_Vx, target_Vy, target_Wz, R, L, alphas);

% 3. Apply safety scaling (calls function from 03_Control)
final_speeds = apply_motor_limits(raw_speeds, max_rad_s);

disp('Final Motor Speeds [FR; FL; BL; BR] in rad/s:');
disp(final_speeds);
