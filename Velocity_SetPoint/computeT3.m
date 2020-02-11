function [ T3 ] = computeT3( T1, a0, j_max )
	T3 = a0 / j_max + T1;
	T3 = max(T3, 0);
end

