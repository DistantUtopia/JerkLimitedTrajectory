function [ T4 ] = computeT4( delt_p_acc, delt_p_dec, v_i, delt_p )
	T4 = (delt_p - delt_p_acc - delt_p_dec) / v_i;
	T4 = max(T4, 0);
end