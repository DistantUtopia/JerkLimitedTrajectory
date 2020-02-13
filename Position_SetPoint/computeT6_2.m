function [ T6 ] = computeT6_2( T567, T5, T7 )
	T6 = T567 - T5 - T7;
	T6 = max(T6, 0);
end

