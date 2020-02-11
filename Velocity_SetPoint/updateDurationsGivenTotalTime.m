function [ T1, T2, T3 ] = updateDurationsGivenTotalTime( T123, direction, max_jerk, max_accel, vel_sp, state_a, state_v )
    jerk_max_T1 = direction * max_jerk;
    delta_v = vel_sp - state_v;
    
    T1 = computeT1_2(T123, state_a, delta_v, jerk_max_T1, max_accel);
    T3 = computeT3(T1, state_a, jerk_max_T1);
    T2 = computeT2_2(T123, T1, T3);
    
end

