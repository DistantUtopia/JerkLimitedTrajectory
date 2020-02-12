clc;clear;close all;
local_time = 0;

%结构体声明
state.p = 0.0;
state.v = 0.0;
state.a = 0.0;
state.j = 0.0;
state.p_sp = 0.0;
state.v_sp = 0.0;
state.a_sp = 0.0;
state.v_max = 0.0;
state.a_max = 0.0;
state.j_max = 0.0;
state.T1 = 0;
state.T2 = 0;
state.T3 = 0;
state.T4 = 0;
state.T5 = 0;
state.T6 = 0;
state.T7 = 0;
state.log_p = [];
state.log_v = [];
state.log_a = [];
state.log_j = [];
state.log_t = [];
state.log_v_sp = [];
state.direction = 0;

%x,y,z三轴状态量初始化
state(1).p = 5;
state(1).v = 1;
state(1).a = 0;
state(1).j = 0;
state(1).p_sp = 0.0;
state(1).v_sp = 2;
state(1).a_sp = 0.0;
state(1).v_max = 5.0;
state(1).a_max = 5.0;
state(1).j_max = 10.0;
state(1).T1 = 0;
state(1).T2 = 0;
state(1).T3 = 0;
state(1).T4 = 0;
state(1).T5 = 0;
state(1).T6 = 0;
state(1).T7 = 0;
state(1).direction = 0;

state(2).p = 5;
state(2).v = 1;
state(2).a = 0;
state(2).j = 0;
state(2).p_sp = 0.0;
state(2).v_sp = 1;
state(2).a_sp = 0.0;
state(2).v_max = 5.0;
state(2).a_max = 5.0;
state(2).j_max = 10.0;
state(2).T1 = 0;
state(2).T2 = 0;
state(2).T3 = 0;
state(2).T4 = 0;
state(2).T5 = 0;
state(2).T6 = 0;
state(2).T7 = 0;
state(2).direction = 0;

state(3).p = 10;
state(3).v = 1;
state(3).a = 0;
state(3).j = 0;
state(3).p_sp = 0.0;
state(3).v_sp = 3;
state(3).a_sp = 0.0;
state(3).v_max = 5.0;
state(3).a_max = 2.0;
state(3).j_max = 5.0;
state(3).T1 = 0;
state(3).T2 = 0;
state(3).T3 = 0;
state(3).T4 = 0;
state(3).T5 = 0;
state(3).T6 = 0;
state(3).T7 = 0;
state(3).direction = 0;

dt = 0.0025;
dt_stick_update = 0.02;
time_stretch = 1.0;
n_sample_stick = 100;
n_index = 1;
n_stick = 1;
total_time = (n_sample_stick + 1) * dt_stick_update;

% 模拟三角波摇杆输入量
% for n = 1 : n_sample_stick + 1
%     if n <= n_sample_stick / 2 
%         state(1).log_v_sp = 2 * (n - 1) / n_sample_stick * state(1).v_max;
%         state(2).log_v_sp = 2 * (n - 1) / n_sample_stick * state(2).v_max;
%         state(3).log_v_sp = 2 * (n - 1) / n_sample_stick * state(3).v_max;
%     else
%         state(1).log_v_sp = (1 + (n_sample_stick / 2 - (n - 1)) * 2 / n_sample_stick) * state(1).v_max;
%         state(2).log_v_sp = (1 + (n_sample_stick / 2 - (n - 1)) * 2 / n_sample_stick) * state(2).v_max;
%         state(3).log_v_sp = (1 + (n_sample_stick / 2 - (n - 1)) * 2 / n_sample_stick) * state(3).v_max;
%     end
% end

for i = 1 : 3
%     [state(i).T1, state(i).T2, state(i).T3, local_time, state(i).direction ] = updateDurations_Position_Setpoint( state(i).v_sp, state(i).a, state(i).v, state(i).j_max, state(i).a_max, state(i).v_max);
    [state(i).T1, state(i).T2, state(i).T3, local_time, state(i).direction ] = updateDurations_Velocity_Setpoint( state(i).v_sp, state(i).a, state(i).v, state(i).j_max, state(i).a_max, state(i).v_max);
end
state = timeSynchronization(state, 2);
t123 = state(1).T1 + state(1).T2 + state(1).T3;
n_steps = ceil(t123 / dt);
% n_steps = ceil(t123 / dt) + (n_sample_stick + 1) * 8 + 400;

for i = 1 : n_steps
    for k = 1 : 3  
        [state(k).p, state(k).v, state(k).a, state(k).j] = updateTraj( dt, time_stretch, local_time, state(k).T1, state(k).T2, state(k).T3, state(k).a, state(k).v, state(k).p, state(k).direction, state(k).j_max );
        state(k).log_p(n_index) = state(k).p;
        state(k).log_v(n_index) = state(k).v;
        state(k).log_a(n_index) = state(k).a;
        state(k).log_j(n_index) = state(k).j;
        state(k).log_t(n_index) = n_index * dt;
        state(k).log_v_sp(n_index) = state(k).v_sp;
%         [state(k).T1, state(k).T2, state(k).T3, local_time, state(k).direction ] = updateDurations_Position_Setpoint( state(k).v_sp, state(k).a, state(k).v, state(k).j_max, state(k).a_max, state(k).v_max);
        [state(k).T1, state(k).T2, state(k).T3, local_time, state(k).direction ] = updateDurations_Velocity_Setpoint( state(k).v_sp, state(k).a, state(k).v, state(k).j_max, state(k).a_max, state(k).v_max);
    end
    state = timeSynchronization(state, 2);
    n_index = n_index + 1;
%定周期改变速度期望设定值     
%     if mod(n_index , 8) == 0 && n_stick <= (n_sample_stick + 1)
%         state(1).v_sp = log_stick(1, n_stick);
%         state(2).v_sp = log_stick(2, n_stick);
%         state(3).v_sp = log_stick(3, n_stick);
%         [state(k).T1, state(k).T2, state(k).T3, local_time, state(k).direction ] = updateDurations_Velocity_Setpoint( state(k).v_sp, state(k).a, state(k).v, state(k).j_max, state(k).a_max, state(k).v_max);
%         state = timeSynchronization(state, 2);
%         t123 = state(1).T1 + state(1).T2 + state(1).T3;
%         n_stick = n_stick + 1;
%     end 
end

figure(1);
plot(state(1).log_t, state(1).log_p, 'r');
hold on;
plot(state(1).log_t, state(1).log_v, 'g');
hold on;
plot(state(1).log_t, state(1).log_a, 'b');
hold on;
plot(state(1).log_t, state(1).log_j, 'm');
% hold on;
% plot(log_t(1, :), log_v_sp(1, :), 'k');
legend('pos_x','vel_x','acc_x','jerk_x');
grid on;

figure(2);
plot(state(2).log_t, state(2).log_p, 'r');
hold on;
plot(state(2).log_t, state(2).log_v, 'g');
hold on;
plot(state(2).log_t, state(2).log_a, 'b');
hold on;
plot(state(2).log_t, state(2).log_j, 'm');
% hold on;
% plot(log_t(2, :), log_v_sp(2, :), 'k');
legend('pos_y','vel_y','acc_y','jerk_y');
grid on;

figure(3);
plot(state(3).log_t, state(3).log_p, 'r');
hold on;
plot(state(3).log_t, state(3).log_v, 'g');
hold on;
plot(state(3).log_t, state(3).log_a, 'b');
hold on;
plot(state(3).log_t, state(3).log_j, 'm');
% hold on;
% plot(log_t(3, :), log_v_sp(3, :), 'k');
legend('pos_z','vel_z','acc_z','jerk_z');
grid on;