function [ T1, T2, T3 ] = updateDurationsMinimizeTotalTime( direction, max_jerk, max_accel, vel_sp, state_a, state_v )
    jerk_max_T1 = direction * max_jerk;
    delta_v = vel_sp - state_v;
    
    T1 = computeT1_1(state_a, delta_v, jerk_max_T1, max_accel);
    T3 = computeT3(T1, state_a, jerk_max_T1);
    T2 = computeT2_1(T1, T3, state_a, delta_v, jerk_max_T1);
    
end

