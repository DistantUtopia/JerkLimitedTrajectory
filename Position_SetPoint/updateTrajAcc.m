function [ p, v, a, j ] = updateTrajAcc( dt, time_stretch, local_time, T1, T2, T3, a0, v0, p0, direction, max_jerk )
    time_stretch = 1.0;
    local_time = local_time + dt * time_stretch;
    t_remain = local_time;
    
    t1 = min(t_remain, T1);
    [p, v, a, j] = evaluatePoly(max_jerk, a0, v0, p0, t1, direction );
    t_remain = t_remain - t1;

    if (t_remain > 0) 
        t2 = min(t_remain, T2);
		[p, v, a, j] = evaluatePoly(0, a, v, p, t2, 0);
		t_remain = t_remain - t2;
    end

    if (t_remain > 0) 
		t3 = min(t_remain, T3);
		[p, v, a, j] = evaluatePoly(max_jerk, a, v, p, t3, -direction);
		t_remain = t_remain - t3;
    end

    if (t_remain > 0) 
		[p, v, a, j] = evaluatePoly(0, 0, v, p, t_remain, 0);
    end
end

