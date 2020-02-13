function [ T5_new ] = saturateT5ForAccel( a0, j_max, T5, a_max )
    accel_T5 = a0 + j_max * T5;
    T5_new = T5;
    
    if (accel_T5 > a_max) 
		T5_new = (a_max - a0) / j_max;
    else
        if (accel_T5 < -a_max) 
            T5_new = (-a_max - a0) / j_max;
        end
    end
end

