function [ p, v, a, j, timestamp ] = updateTraj( dt, timestamp, T1, T2, T3, T4, T5, T6, T7, a0, v0, p0, direction_acc, direction_dec, max_jerk )
    timestamp = timestamp + dt;
    p = 0;
    a = 0;
    v = 0;
    j = 0;
    if timestamp < (T1 + T2 + T3)
        %update accleration trajectory
        [ p, v, a, j ] = updateTrajAcc( timestamp, T1, T2, T3, a0, v0, p0, direction_acc, max_jerk );
    end

    if timestamp >= (T1 + T2 + T3) && timestamp < (T1 + T2 + T3 + T4)
        %update cruise trajectory
        [ p, v, a, j ] = updateTrajAcc( T1 + T2 + T3, T1, T2, T3, a0, v0, p0, direction_acc, max_jerk );
        [ p, v, a, j ] = updateTrajCruise( timestamp - (T1 + T2 + T3), T4, 0, v, p, 0, max_jerk );
    end
    if timestamp >= (T1 + T2 + T3 + T4)
        %update decleration trajectory
        [ p, v, a, j ] = updateTrajAcc( T1 + T2 + T3, T1, T2, T3, a0, v0, p0, direction_acc, max_jerk );
        [ p, v, a, j ] = updateTrajCruise( T4, T4, 0, v, p, 0, max_jerk );
        [ p, v, a, j ] = updateTrajAcc( timestamp - (T1 + T2 + T3 + T4), T5, T6, T7, a, v, p, direction_dec, max_jerk );
    end
end

