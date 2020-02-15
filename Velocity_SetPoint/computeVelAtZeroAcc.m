function [ vel_zero_acc ] = computeVelAtZeroAcc( state_v, state_a, max_jerk )
    vel_zero_acc = state_v;
    if abs(state_a) > 1.192093e-007
        j_zero_acc = - sign(state_a) * max_jerk;
        t_zero_acc = - state_a / j_zero_acc;
        vel_zero_acc = state_v + state_a * t_zero_acc + 0.5 * j_zero_acc * t_zero_acc * t_zero_acc;
    end
end

