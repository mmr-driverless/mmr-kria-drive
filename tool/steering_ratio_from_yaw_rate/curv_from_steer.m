function [k] = curv_from_steer(steering_wheel_ang_rad, steering_ratio, L, FBAL)
    % Distance from rear/front axes to CoM
    LR = L * FBAL;
    LF = L * (1 - FBAL);
    
    u = steering_wheel_ang_rad / steering_ratio;

    beta = atan(LR * tan(u) / L);
    k = (tan(u) .* cos(beta) - sin(beta)) / LF;
end