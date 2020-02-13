function [ T1, T2, T3, delt_p_acc, p, v, a, j ] = updateDurationsMinimizeTotalTimeAccleration( direction, max_jerk, max_accel, vel_sp, state_a, state_v, state_p )
    jerk_max_T1 = direction * max_jerk;
    delta_v = vel_sp - state_v;
    
    T1 = computeT1_1(state_a, delta_v, jerk_max_T1, max_accel);
    T3 = computeT3(T1, state_a, jerk_max_T1);
    T2 = computeT2_1(T1, T3, state_a, delta_v, jerk_max_T1);
    
    delt_p_acc = 0;
    local_time = 0;
    [p, v, a, j] = updateTrajAcc( T1 + T2 + T3, 1.0, local_time, T1, T2, T3, state_a, state_v, state_p, direction, max_jerk );
    delt_p_acc = p - state_p;
end

