function [ direction ] = computeDirection( vel_sp, state_a, state_v, max_jerk )
    vel_zero_acc = computeVelAtZeroAcc(state_v, state_a, max_jerk);
    
    direction = sign(vel_sp - vel_zero_acc);
    
    if direction == 0
        direction = sign(state_a);
    end
end

