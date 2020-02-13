function [ p, v, a, j ] = updateTrajCruise( dt, time_stretch, local_time, T4, a0, v0, p0, direction, max_jerk )
    time_stretch = 1.0;
    local_time = local_time + dt * time_stretch;
    t_remain = local_time;
    
    t4 = min(t_remain, T4);
    [p, v, a, j] = evaluatePoly( max_jerk, a0, v0, p0, t4, direction );
    t_remain = t_remain - t4;

    if (t_remain > 0) 
		[p, v, a, j] = evaluatePoly( 0, 0, v, p, t_remain, 0 );
    end
end

