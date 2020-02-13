function [ T5 ] = computeT5_1(  a0, v3, j_max, a_max  )
    delta = 2 * a0 * a0 + 4 * j_max * v3;
    
    if delta < 0
        T5 = 0;
        return;
    end
    
    sqrt_delta = sqrt(delta);
    T5_plus = (-a0 + 0.5 * sqrt_delta) / j_max;
    T5_minus = (-a0 - 0.5 * sqrt_delta) / j_max;
    
    T7_plus = a0 / j_max + T5_plus;
    T7_minus = a0 / j_max + T5_minus;
    
    T5 = 0;
    
    if (T5_plus >= 0 && T7_plus >= 0) 
		T5 = T5_plus;
    else
        if (T5_minus >= 0 && T7_minus >= 0)
            T5 = T5_minus;
        else
            
        end    
    end
    
    T5 = saturateT5ForAccel(a0, j_max, T5, a_max);
    
    T5 = max(T5, 0);
end

