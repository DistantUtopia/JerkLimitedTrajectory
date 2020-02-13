function [ p, v, a, j ] = updateTraj( dt, time_stretch, local_time, total_time, T1, T2, T3, T4, T5, T6, T7, a0, v0, p0, direction_acc, direction_dec, max_jerk )
    total_time = total_time + dt;
    p = 0;
    a = 0;
    v = 0;
    j = 0;
    if total_time < (T1 + T2 + T3)
        %update accleration trajectory
        [ p, v, a, j ] = updateTrajAcc( dt, time_stretch, local_time, T1, T2, T3, a0, v0, p0, direction_acc, max_jerk );
    end

    if total_time >= (T1 + T2 + T3) && total_time < (T1 + T2 + T3 + T4)
        %update cruise trajectory
        [ p, v, a, j ] = updateTrajCruise( dt, time_stretch, local_time, T4, 0, v0, p0, 0, max_jerk );
    end
    if total_time >= (T1 + T2 + T3 + T4)
        %update decleration trajectory
        [ p, v, a, j ] = updateTrajAcc( dt, time_stretch, local_time, T5, T6, T7, a0, v0, p0, direction_dec, max_jerk );
    end
end

