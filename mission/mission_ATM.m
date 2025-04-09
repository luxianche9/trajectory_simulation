clear; clc; close all;

%% 仿真时间设置
t0 = 0;
dt = 0.01;
tf = 100;

%% 仿真参数设置
% 导弹初始状态
% V, theta, phi_V, x, y, z, omega_x, omega_y, omega_z, nu, phi, gama, m
V0 = 20;
theta0 = deg2rad(18);
phi_V0 = deg2rad(0);
xm0 = 0;
ym0 = 20;
zm0 = 0;
omega_x0 = 0;
omega_y0 = 0;
omega_z0 = 0;
nu0 = deg2rad(18);
phi0 = deg2rad(0);
gama0 = deg2rad(0);
m0 = 52.38;
% 目标初始状态
% y = [V, theta, phi_V, x, y, z]
V_t0 = 25.5;
theta_t0 = deg2rad(0);
phi_t0 = atan(5/15);
x_t0 = 4000;
y_t0 = 0;
z_t0 = 0;
% 控制器(积分器)状态
int_n_y2 = 0;
int_n_z2 = 0;

y0 = [V0; theta0; phi_V0; xm0; ym0; zm0; omega_x0; omega_y0; omega_z0; nu0; phi0; gama0; m0; ...
    V_t0; theta_t0; phi_t0; x_t0; y_t0; z_t0; int_n_y2; int_n_z2];

%% 导弹初始化
missile = Missile(t0, dt, tf);
target = Target('straight');

%% 数值求解动态方程
ode = @(t, y) ode_wrap(t, y, missile, target);

event = @(t, y) event_wrap(t, y, missile);

[t, y] = ode_EPC(t0, dt, tf, y0, ode, event);

%% 结果分析
result_analysis(missile, t, y);

%% ode, event
function dy_dt = ode_wrap(t, y, missile, target)
    % ode45求解
    missile_states = y(1:13);
    target_states = y(14:19);
    int_n_y2 = y(20);
    int_n_z2 = y(21);
    i = round((t - missile.t0) / missile.dt) + 1;
    n_y2 = missile.recode.n_y2(i);
    n_z2 = missile.recode.n_z2(i);

    % 制导
    [n_y2_cmd, n_z2_cmd, nu_cmd, flag] = missile.Missile_Guidance(t, missile_states, target_states);

    if isnan(n_y2_cmd) || isnan(n_z2_cmd)
        n_y2_cmd = 0;
        n_z2_cmd = 0;
    end
    diff_n_y2 = n_y2_cmd - n_y2;
    diff_n_z2 = n_z2_cmd - n_z2;

    % 控制器
    [delta_y, delta_z] = missile.Missile_Control(missile_states, int_n_y2, int_n_z2, nu_cmd, flag);
    dy_dt_missile = missile.Missile_Dynamics(t, missile_states, delta_y, delta_z);
    dy_dt_target = target.Target_Dynamics(target_states);
    dy_dt_controler = missile.Missile_Controler_Dynamics(diff_n_y2, diff_n_z2);

    dy_dt = [dy_dt_missile; dy_dt_target; dy_dt_controler];
end

function value = event_wrap(t, y, missile)
    % [value, isterminal, direction]
    % isterminal = 1; % 终止仿真
    % direction = 0;
    value = 0;

    missile_pos = y(4:6); % 导弹位置
    target_pos = y(17:19); % 目标位置

    if missile.Hit_Ground(missile_pos) == 1
        value = 1; % 触发终止
    end
    if missile.Hit_Target(missile_pos, target_pos) == 1
        value = 1; % 触发终止
    end

    % value = ~value;
end