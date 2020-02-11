function [ T1_new ] = saturateT1ForAccel( a0, j_max, T1, a_max )
    accel_T1 = a0 + j_max * T1;
    T1_new = T1;
    
    if (accel_T1 > a_max) 
		T1_new = (a_max - a0) / j_max;
    else
        if (accel_T1 < -a_max) 
            T1_new = (-a_max - a0) / j_max;
        end
    end
end

