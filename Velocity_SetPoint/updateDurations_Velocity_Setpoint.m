function [ T1, T2, T3, local_time, direction ] = updateDurations_Velocity_Setpoint( vel_sp, state_a, state_v, max_jerk, max_accel, max_velocity )
    if vel_sp > max_velocity
        vel_sp = max_velocity;
    end
    if vel_sp < - max_velocity
        vel_sp = - max_velocity;
    end
    
    local_time = 0.0;
    
    direction = computeDirection(vel_sp, state_a, state_v, max_jerk);
    
    if direction ~= 0
        [T1, T2, T3] = updateDurationsMinimizeTotalTime(direction, max_jerk, max_accel, vel_sp, state_a, state_v);
    else
        T1 = 0;
        T2 = 0;
        T3 = 0;
    end
end

