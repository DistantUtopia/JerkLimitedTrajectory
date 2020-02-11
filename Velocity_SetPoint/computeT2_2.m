function [ T2 ] = computeT2_2( T123, T1, T3 )
	T2 = T123 - T1 - T3;
	T2 = max(T2, 0);
end

