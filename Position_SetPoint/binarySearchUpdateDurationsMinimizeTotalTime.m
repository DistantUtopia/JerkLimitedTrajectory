function [ T1, T2, T3, T4, T5, T6, T7, local_time, direction_acc, direction_dec ] = binarySearchUpdateDurationsMinimizeTotalTime( j_max, a0, a_max, v0, v_max, p0, pt )
    %初始化速度、位置参数
    delt_p = pt - p0;
    v_now = sign(delt_p) * v_max;
    v_s = 0;
    v_e = sign(delt_p) * v_max;
    
    MeetTheSetPoint = 0;
    delt_p_thr = 0.1;
    
    iterCounter = 1;
    
    while(MeetTheSetPoint ~= 1)
        %更新时间及位置增量
        [ T1, T2, T3, T4, T5, T6, T7, local_t, dir_acc, dir_dec, deltP ] = updateDurations_Position_Setpoint( v_now, a0, v0, j_max, a_max, v_max, p0, pt );
        direction_acc = dir_acc;
        direction_dec = dir_dec;
        local_time = local_t;
        if abs(deltP - delt_p) <= delt_p_thr || iterCounter > 100
            MeetTheSetPoint = 1;
        else
            MeetTheSetPoint = 0;
        end
        %更新v_now值重新进行时间计算
        if abs(deltP) > abs(delt_p)
            v_s = 0;
            v_e = v_now;
            v_now = (v_s + v_e) / 2;
        else
            if abs(deltP) < abs(delt_p)
                v_s = v_now;
                v_e = sign(delt_p) * v_max;
                v_now = (v_s + v_e) / 2;
            else

            end
        end
        iterCounter = iterCounter + 1;
    end   
end

