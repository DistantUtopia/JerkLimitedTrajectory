function [ T5, T6, T7, delt_p_dec, p, v, a, j ] = updateDurationsMinimizeTotalTimeDecleration( direction, max_jerk, max_accel, vel_sp, state_a, state_v, state_p )
    jerk_max_T5 = direction * max_jerk;
    delta_v = vel_sp - state_v;
    
    T5 = computeT5_1(state_a, delta_v, jerk_max_T5, max_accel);
    T7 = computeT7(T5, state_a, jerk_max_T5);
    T6 = computeT6_1(T5, T7, state_a, delta_v, jerk_max_T5);
    
    delt_p_dec = 0;
    local_time = 0;
    [p, v, a, j] = updateTrajDec( T5 + T6 + T7, 1.0, local_time, T5, T6, T7, state_a, state_v, state_p, direction, max_jerk );
    delt_p_dec = p - state_p;
end
