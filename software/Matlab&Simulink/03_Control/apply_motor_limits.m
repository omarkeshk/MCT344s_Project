function W_motors_scaled = apply_motor_limits(W_wheels_raw, max_rad_s)
    max_calc_speed = max(abs(W_wheels_raw));
    
    if max_calc_speed > max_rad_s
        scale_factor = max_rad_s / max_calc_speed;
        W_motors_scaled = W_wheels_raw * scale_factor;
        fprintf('Warning: Scaled motors down by %.2f\n', scale_factor);
    else
        W_motors_scaled = W_wheels_raw;
    end
end
