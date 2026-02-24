% test_pure_rotation.m
% Tests the in-place rotation for the GREEN Pickup/Drop-off zone

disp('--- Running Station C (Pure Rotation) Test ---');

% 1. Define target speeds: Rotate counter-clockwise at 1.0 rad/s
target_Vx = 0.0; 
target_Vy = 0.0; 
target_Wz = 1.0;

% 2. Calculate raw wheel speeds (calls function from 02_Kinematics)
raw_speeds = inverse_kinematics(target_Vx, target_Vy, target_Wz, R, L, alphas);

% 3. Apply safety scaling (calls function from 03_Control)
final_speeds = apply_motor_limits(raw_speeds, max_rad_s);

disp('Final Motor Speeds [FR; FL; BL; BR] in rad/s:');
disp(final_speeds);
