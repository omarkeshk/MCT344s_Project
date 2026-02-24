function W_wheels = inverse_kinematics(Vx, Vy, Wz, R, L, alphas)
    % Calculates raw wheel speeds for an omni-base
    J_inv = (1/R) * [ cos(alphas(1)), sin(alphas(1)), L;
                      cos(alphas(2)), sin(alphas(2)), L;
                      cos(alphas(3)), sin(alphas(3)), L;
                      cos(alphas(4)), sin(alphas(4)), L ];
                      
    V_target = [Vx; Vy; Wz];
    W_wheels = J_inv * V_target;
end
