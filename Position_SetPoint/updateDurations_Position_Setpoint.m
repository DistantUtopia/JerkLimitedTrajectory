function [ T1, T2, T3, T4, T5, T6, T7, local_time, direction_acc, direction_dec, deltP ] = updateDurations_Position_Setpoint( vel_sp, state_a, state_v, max_jerk, max_accel, max_velocity, p0, pt )
    p_acc = 0; v_acc = 0; a_acc = 0; j_acc = 0; delt_p_acc = 0;
    p_dec = 0; v_dec = 0; a_dec = 0; j_dec = 0; delt_p_dec = 0;
    deltP = 0;    
    delt_p = pt - p0;
    if vel_sp > max_velocity
        vel_sp = max_velocity;
    end
    if vel_sp < - max_velocity
        vel_sp = - max_velocity;
    end
    
    local_time = 0.0;
    
    direction_acc = computeDirection(vel_sp, state_a, state_v, max_jerk); 
    if direction_acc ~= 0
        [T1, T2, T3, delt_p_acc, p_acc, v_acc, a_acc, j_acc] = updateDurationsMinimizeTotalTimeAccleration(direction_acc, max_jerk, max_accel, vel_sp, state_a, state_v, p0);
    else
        T1 = 0;
        T2 = 0;
        T3 = 0;
    end
    
    state_a = 0;
    state_v = vel_sp;
    vel_sp = 0;
    direction_dec = computeDirection(vel_sp, state_a, state_v, max_jerk); 
    if direction_dec ~= 0
        [T5, T6, T7, delt_p_dec, p_dec, v_dec, a_dec, j_dec] = updateDurationsMinimizeTotalTimeDecleration(direction_dec, max_jerk, max_accel, vel_sp, a_acc, v_acc, p_acc);
    else
        T5 = 0;
        T6 = 0;
        T7 = 0;
    end
    
    T4 = computeT4( delt_p_acc, delt_p_dec, state_v, delt_p );
    
    delt_p_cruise = 0;
    [p_cruise, v_cruise, a_cruise, j_cruise] = updateTrajCruise( T4, 1.0, local_time, T4, 0, v_acc, p_acc, 0, max_jerk );
    delt_p_cruise = p_cruise - p_acc;
    
    deltP = delt_p_acc + delt_p_dec + delt_p_cruise;
end

