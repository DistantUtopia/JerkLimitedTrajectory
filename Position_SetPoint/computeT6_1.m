function [ T6 ] = computeT6_1( T5, T7, a0, v3, j_max )
    T6 = 0;
    den = a0 + j_max * T5;
    
    if(abs(den) > 0)
        T6 = (-0.5 * T5 * T5 * j_max - T5 * T7 * j_max - T5 * a0 + 0.5 * T7 * T7 * j_max - T7 * a0 + v3) / den;
    end

    T6 = max(T6, 0);                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                      
end

