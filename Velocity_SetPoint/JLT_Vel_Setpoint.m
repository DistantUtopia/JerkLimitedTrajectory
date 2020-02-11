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
state.direction = 0;

stick.value = 0;

%x,y,z三轴状态量初始化
state(1).p = 10;
state(1).v = 1;
state(1).a = 0;
state(1).j = 0;
state(1).p_sp = 0.0;
state(1).v_sp = 2.0;
state(1).a_sp = 0.0;
state(1).v_max = 5.0;
state(1).a_max = 5.0;
state(1).j_max = 10.0;
state(1).T1 = 0;
state(1).T2 = 0;
state(1).T3 = 0;
state(1).direction = 0;

stick(1).value = 0;

state(2).p = 10;
state(2).v = 1;
state(2).a = 0;
state(2).j = 0;
state(2).p_sp = 0.0;
state(2).v_sp = 3.0;
state(2).a_sp = 0.0;
state(2).v_max = 5.0;
state(2).a_max = 5.0;
state(2).j_max = 10.0;
state(2).T1 = 0;
state(2).T2 = 0;
state(2).T3 = 0;
state(2).direction = 0;

stick(2).value = 0;

state(3).p = 10;
state(3).v = 1;
state(3).a = 0;
state(3).j = 0;
state(3).p_sp = 0.0;
state(3).v_sp = 5.0;
state(3).a_sp = 0.0;
state(3).v_max = 5.0;
state(3).a_max = 2.0;
state(3).j_max = 5.0;
state(3).T1 = 0;
state(3).T2 = 0;
state(3).T3 = 0;
state(3).direction = 0;

stick(3).value = 0;

log_p = zeros(3,1);
log_v = zeros(3,1);
log_a = zeros(3,1);
log_j = zeros(3,1);
log_t = zeros(3,1);
log_stick = zeros(3,1);
log_v_sp = zeros(3,1);

dt = 0.0025;
dt_stick_update = 0.02;
time_stretch = 1.0;
n_sample_stick = 100;
n_index = 1;
n_stick = 1;
total_time = (n_sample_stick + 1) * dt_stick_update;
absolute_time = 0;

% 模拟三角波摇杆输入量
for n = 1 : n_sample_stick + 1
    if n <= n_sample_stick / 2 
        stick(1).value = 2 * (n - 1) / n_sample_stick * state(1).v_max;
        stick(2).value = 2 * (n - 1) / n_sample_stick * state(2).v_max;
        stick(3).value = 2 * (n - 1) / n_sample_stick * state(3).v_max;
    else
        stick(1).value = (1 + (n_sample_stick / 2 - (n - 1)) * 2 / n_sample_stick) * state(1).v_max;
        stick(2).value = (1 + (n_sample_stick / 2 - (n - 1)) * 2 / n_sample_stick) * state(2).v_max;
        stick(3).value = (1 + (n_sample_stick / 2 - (n - 1)) * 2 / n_sample_stick) * state(3).v_max;
    end
    log_stick(1, n) = stick(1).value;
    log_stick(2, n) = stick(2).value;
    log_stick(3, n) = stick(3).value;
end

for i = 1 : 3
    [state(i).T1, state(i).T2, state(i).T3, local_time, state(i).direction ] = updateDurations_Velocity_Setpoint( state(i).v_sp, state(i).a, state(i).v, state(i).j_max, state(i).a_max, state(i).v_max);
end
state = timeSynchronization(state, 2);
t123 = state(1).T1 + state(1).T2 + state(1).T3;
n_steps = ceil(t123 / dt) + (n_sample_stick + 1) * 8 + 400;

for i = 1 : n_steps
    for k = 1 : 3  
        [state(k).p, state(k).v, state(k).a, state(k).j] = updateTraj( dt, time_stretch, local_time, state(k).T1, state(k).T2, state(k).T3, state(k).a, state(k).v, state(k).p, state(k).direction, state(k).j_max );
        log_p(k, n_index) = state(k).p;
        log_v(k, n_index) = state(k).v;
        log_a(k, n_index) = state(k).a;
        log_j(k, n_index) = state(k).j;
        log_t(k, n_index) = n_index * dt;
        log_v_sp(k, n_index) = state(k).v_sp;
        [state(k).T1, state(k).T2, state(k).T3, local_time, state(k).direction ] = updateDurations_Velocity_Setpoint( state(k).v_sp, state(k).a, state(k).v, state(k).j_max, state(k).a_max, state(k).v_max);
    end
    state = timeSynchronization(state, 2);
    n_index = n_index + 1;
    absolute_time = absolute_time + dt;
%定周期改变速度期望设定值     
    if mod(n_index , 8) == 0 && n_stick <= (n_sample_stick + 1)
        state(1).v_sp = log_stick(1, n_stick);
        state(2).v_sp = log_stick(2, n_stick);
        state(3).v_sp = log_stick(3, n_stick);
        [state(k).T1, state(k).T2, state(k).T3, local_time, state(k).direction ] = updateDurations_Velocity_Setpoint( state(k).v_sp, state(k).a, state(k).v, state(k).j_max, state(k).a_max, state(k).v_max);
        state = timeSynchronization(state, 2);
        t123 = state(1).T1 + state(1).T2 + state(1).T3;
        n_stick = n_stick + 1;
    end 
end

figure(1);
plot(log_t(1, :), log_p(1, :), 'r');
hold on;
plot(log_t(1, :), log_v(1, :), 'g');
hold on;
plot(log_t(1, :), log_a(1, :), 'b');
hold on;
plot(log_t(1, :), log_j(1, :), 'm');
hold on;
plot(log_t(1, :), log_v_sp(1, :), 'k');
legend('pos_x','vel_x','acc_x','jerk_x', 'v_sp_x');
grid on;

figure(2);
plot(log_t(2, :), log_p(2, :), 'r');
hold on;
plot(log_t(2, :), log_v(2, :), 'g');
hold on;
plot(log_t(2, :), log_a(2, :), 'b');
hold on;
plot(log_t(2, :), log_j(2, :), 'm');
hold on;
plot(log_t(2, :), log_v_sp(2, :), 'k');
legend('pos_y','vel_y','acc_y','jerk_y', 'v_sp_y');
grid on;

figure(3);
plot(log_t(3, :), log_p(3, :), 'r');
hold on;
plot(log_t(3, :), log_v(3, :), 'g');
hold on;
plot(log_t(3, :), log_a(3, :), 'b');
hold on;
plot(log_t(3, :), log_j(3, :), 'm');
hold on;
plot(log_t(3, :), log_v_sp(3, :), 'k');
legend('pos_z','vel_z','acc_z','jerk_z', 'v_sp_z');
grid on;

figure(4)
plot(log_stick(1, :), 'r');
grid on;



