function [ p, v, a, j ] = evaluatePoly( jerk, a0, v0, x0, t, d )
    jt = d * jerk;
	t2 = t * t;
	t3 = t2 * t;

	j = jt;
	a = a0 + jt * t;
	v = v0 + a0 * t + 0.5 * jt * t2;
	p = x0 + v0 * t + 0.5 * a0 * t2 + 1 / 6 * jt * t3;

end

