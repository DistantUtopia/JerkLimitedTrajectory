function [ T1 ] = computeT1_2( T123, a0, v3, j_max, a_max )
    a = -j_max;
    b = j_max * T123 - a0;
    delta = T123 * T123 * j_max * j_max + 2 * T123 * a0 * j_max - a0 * a0 - 4 * j_max * v3;
    
    if delta < 0
        T1 = 0;
        return;
    end
    
    sqrt_delta = sqrt(delta);
    denominator_inv = 1 / (2 * a);
    T1_plus = max((-b + sqrt_delta) * denominator_inv, 0);
    T1_minus = max((-b - sqrt_delta) * denominator_inv, 0);
    
    T3_plus = a0 / j_max + T1_plus;
    T3_minus = a0 / j_max + T1_minus;
    
    T13_plus = T1_plus + T3_plus;
    T13_minus = T1_minus + T3_minus;
    
    T1 = 0;
    
    if (T13_plus > T123) 
		T1 = T1_minus;
    else
        if (T13_minus > T123) 
		T1 = T1_plus;
        end
    end
    T1 = saturateT1ForAccel(a0, j_max, T1, a_max);
end

