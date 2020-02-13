function [ T7 ] = computeT7( T5, a0, j_max )
    T7 = a0 / j_max + T5;
	T7 = max(T7, 0);
end

