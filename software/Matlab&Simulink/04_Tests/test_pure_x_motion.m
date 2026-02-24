% test_pure_x_strafe.m
% Tests the sideways strafing motion for the Blue Drop-off zone

% 1. Define target speeds (Move right at 0.5 m/s)
target_Vx = 0.5; 
target_Vy = 0.0; 
target_Wz = 0.0;

% 2. Calculate raw speeds (calls the function from 02_Kinematics)
raw_speeds = inverse_kinematics(target_Vx, target_Vy, target_Wz, R, L, alphas);

% 3. Apply gearbox ratio
motor_speeds = raw_speeds * gear_ratio;

% 4. Apply safety scaling (calls the function from 03_Control)
final_speeds = apply_motor_limits(motor_speeds, max_rad_s);

disp('Final commands to send to MCUs (rad/s):');
disp(final_speeds);
