function [state, longest_time] = timeSynchronization( state, n_traj )
    desired_time = 0;
    longest_traj_index = 0;
    longest_time = 0;
    for i = 1 : n_traj
        T123 = state(i).T1 + state(i).T2 + state(i).T3;
        if T123 > desired_time
            desired_time = T123;
            longest_traj_index = i;
        end
    end
    longest_time = desired_time;
    
    if desired_time > 1.192093e-007
        for i = 1 : n_traj
            if i ~= longest_traj_index
                [state(i).T1, state(i).T2, state(i).T3] = updateDurationsGivenTotalTime(desired_time, state(i).direction, state(i).j_max, state(i).a_max, state(i).v_sp, state(i).a, state(i).v);
            end
        end
    end

end

