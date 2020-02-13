function [ T5 ] = computeT5_2( T567, a0, v3, j_max, a_max )
    a = -j_max;
    b = j_max * T567 - a0;
    delta = T567 * T567 * j_max * j_max + 2 * T567 * a0 * j_max - a0 * a0 - 4 * j_max * v3;
    
    if delta < 0
        T5 = 0;
        return;
    end
    
    sqrt_delta = sqrt(delta);
    denominator_inv = 1 / (2 * a);
    T5_plus = max((-b + sqrt_delta) * denominator_inv, 0);
    T5_minus = max((-b - sqrt_delta) * denominator_inv, 0);
    
    T7_plus = a0 / j_max + T5_plus;
    T7_minus = a0 / j_max + T5_minus;
    
    T57_plus = T5_plus + T7_plus;
    T57_minus = T5_minus + T7_minus;
    
    T5 = 0;
    
    if (T57_plus > T567) 
		T5 = T5_minus;
    else
        if (T57_minus > T567) 
		T5 = T5_plus;
        end
    end
    T5 = saturateT5ForAccel(a0, j_max, T5, a_max);
end
