function [ p, v, a, j ] = updateTrajDec( timestamp, T5, T6, T7, a0, v0, p0, direction, max_jerk )
    t_remain = timestamp;
    
    t5 = min(t_remain, T5);
    [p, v, a, j] = evaluatePoly(max_jerk, a0, v0, p0, t5, direction );
    t_remain = t_remain - t5;

    if (t_remain > 0) 
        t6 = min(t_remain, T6);
		[p, v, a, j] = evaluatePoly(0, a, v, p, t6, 0);
		t_remain = t_remain - t6;
    end

    if (t_remain > 0) 
		t7 = min(t_remain, T7);
		[p, v, a, j] = evaluatePoly(max_jerk, a, v, p, t7, -direction);
		t_remain = t_remain - t7;
    end

    if (t_remain > 0) 
		[p, v, a, j] = evaluatePoly(0, 0, v, p, t_remain, 0);
    end
end
