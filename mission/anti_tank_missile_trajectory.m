clear; clc; close all;

%% 仿真时间设置
t0 = 0;
dt = 0.1;
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
x_t0 = 12000;
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

y = y';
length_t = length(t);

%% 提取数据
V_m = y(:, 1); theta = y(:, 2); phi_V = y(:, 3);
x_m = y(:, 4); y_m = y(:, 5); z_m = y(:, 6);
omega_x = y(:, 7); omega_y = y(:, 8); omega_z = y(:, 9);
nu = y(:, 10); phi = y(:, 11); gama = y(:, 12);

n_x2 = missile.recode.n_x2(1:length_t);
n_y2 = missile.recode.n_y2(1:length_t);
n_z2 = missile.recode.n_z2(1:length_t);

x_t = y(:, 17); y_t = y(:, 18); z_t = y(:, 19);

delta_y = missile.recode.delta_y(1:length_t);
delta_z = missile.recode.delta_z(1:length_t);
nu_cmd = missile.recode.nu_cmd(1:length_t);
n_y2_cmd = missile.recode.n_y2_cmd(1:length_t);
n_z2_cmd = missile.recode.n_z2_cmd(1:length_t);

t_plan = missile.t_plan;


%% 1. 三维轨迹图
figure;
pos_m = [x_m'; y_m'; z_m'];
pos_t = [x_t'; y_t'; z_t'];
L = [1 0 0; 0 0 -1; 0 1 0];
pos_m = L * pos_m;
pos_t = L * pos_t;

idx_plan = find(t < t_plan);
idx_guidance = find(t >= t_plan);

hold on;
fig = plot3(pos_m(1, idx_plan), pos_m(2, idx_plan), pos_m(3, idx_plan), 'b-', "DisplayName", '方案飞行段', 'LineWidth', 1.8);
plot3(pos_m(1, idx_guidance), pos_m(2, idx_guidance), pos_m(3, idx_guidance), 'r-', "DisplayName", '比例导引段', 'LineWidth', 1.8);
plot3(pos_t(1, :), pos_t(2,:), pos_t(3,:), 'k--', "DisplayName", '目标轨迹', 'LineWidth', 1.8);
plot3(pos_m(1,end), pos_m(2,end), pos_m(3,end), 'ko', 'MarkerSize', 6, 'MarkerFaceColor', 'y', 'DisplayName', '打击点');
hold off;
xlabel('x_m (m)'); ylabel('z_m (m)'); zlabel('y_m (m)'); title('导弹与目标轨迹');
grid on; view(3); legend('show'); axis equal;

%% 2. 角速度
figure;
plot(t, omega_x, 'LineWidth', 1.8); grid on; xlabel('时间 (s)'); ylabel('\omega_x'); title('角速度 \omega_x');
figure;
plot(t, omega_y, 'LineWidth', 1.8); grid on; xlabel('时间 (s)'); ylabel('\omega_y'); title('角速度 \omega_y');

figure;
plot(t, omega_z, 'LineWidth', 1.8); grid on; xlabel('时间 (s)'); ylabel('\omega_z'); title('角速度 \omega_z');

%% 3. 姿态角
figure;
plot(t, rad2deg(gama), 'LineWidth', 1.8); grid on; xlabel('时间 (s)'); ylabel('\gamma (°)'); title('滚转角 \gamma');

figure;
plot(t, rad2deg(phi), 'LineWidth', 1.8); grid on; xlabel('时间 (s)'); ylabel('\phi (°)'); title('偏航角 \phi');

figure;
plot(t, rad2deg(nu), 'LineWidth', 1.8); hold on; plot(t, rad2deg(nu_cmd), '--', 'LineWidth', 1.5); grid on;
xlabel('时间 (s)'); ylabel('\nu (°)'); title('俯仰角 \nu'); legend('实际', '指令');

%% 4. 舵偏角
figure;
plot(t, rad2deg(delta_y), 'LineWidth', 1.8); grid on; xlabel('时间 (s)'); ylabel('\delta_y (°)'); title('俯仰舵');

figure;
plot(t, rad2deg(delta_z), 'LineWidth', 1.8); grid on; xlabel('时间 (s)'); ylabel('\delta_z (°)'); title('偏航舵');

%% 5. 过载
figure;
plot(t, n_x2, 'LineWidth', 1.8); grid on; xlabel('时间 (s)'); ylabel('n_{x2}'); title('纵向过载');

figure;
plot(t, n_y2, 'LineWidth', 1.8); hold on; plot(t, n_y2_cmd, '--', 'LineWidth', 1.5); grid on;
xlabel('时间 (s)'); ylabel('n_{y2}'); title('俯仰向过载'); legend('实际', '指令');

figure;
plot(t, n_z2, 'LineWidth', 1.8); hold on; plot(t, n_z2_cmd, '--', 'LineWidth', 1.5); grid on;
xlabel('时间 (s)'); ylabel('n_{z2}'); title('偏航向过载'); legend('实际', '指令');

%% 6. 速度
figure;
plot(t, rad2deg(phi_V), 'LineWidth', 1.8); grid on; xlabel('时间 (s)'); ylabel('\phi_v (°)'); title('速度偏角 \phi_v');

figure;
plot(t, rad2deg(theta), 'LineWidth', 1.8); grid on; xlabel('时间 (s)'); ylabel('\theta (°)'); title('速度倾角 \theta');

figure;
plot(t, V_m, 'LineWidth', 1.8); grid on; xlabel('时间 (s)'); ylabel('V_m (m/s)'); title('导弹速度');

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