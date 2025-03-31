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
x_t0 = 12000;
y_t0 = 0;
z_t0 = 0;

y0 = [V0; theta0; phi_V0; xm0; ym0; zm0; omega_x0; omega_y0; omega_z0; nu0; phi0; gama0; m0; ...
    V_t0; theta_t0; phi_t0; x_t0; y_t0; z_t0];

%% 导弹初始化
missile = Missile(t0, dt, tf);
target = Target('straight');

%% 数值求解动态方程
t = t0:dt:tf;

ode = @(t, y) ode_wrap(t, y, missile, target);

event = @(t, y) event_wrap(t, y, missile);

options = odeset('Events', event, 'RelTol', 1e-6, 'AbsTol', 1e-9);

[t, y] = ode45(ode, t, y0, options);

%% 数据提取
% 导弹状态
% V, theta, phi_V, x, y, z, omega_x, omega_y, omega_z, nu, phi, gama, m
V_m = y(:, 1);
theta = y(:, 2);
phi_V = y(:, 3);
x_m = y(:, 4);
y_m = y(:, 5);
z_m = y(:, 6);
omega_x = y(:, 7);
omega_y = y(:, 8);
omega_z = y(:, 9);
nu = y(:, 10);
phi = y(:, 11);
gama = y(:, 12);
m = y(:, 13);
% 目标状态
% V, theta, phi_V, x, y, z
V_t = y(:, 14);
theta_t = y(:, 15);
phi_t = y(:, 16);
x_t = y(:, 17);
y_t = y(:, 18);
z_t = y(:, 19);

%% 数据可视化
figure; hold on;
pos_m = [x_m'; y_m'; z_m'];
pos_t = [x_t'; y_t'; z_t'];
L = [1 0 0;
    0 0 -1;
    0 1 0];
pos_m = L * pos_m;
pos_t = L * pos_t;
plot3(pos_m(1, :), pos_m(2,:), pos_m(3,:), 'b-', "DisplayName", 'missel');
plot3(pos_t(1, :), pos_t(2,:), pos_t(3,:), 'r-', "DisplayName", 'target');
xlabel('xm (m)'); ylabel('zm (m)'); zlabel('ym (m)'); title('Missile&Target Trajectory');
grid on; view(3); legend('show'); axis equal;

figure;
subplot(4,3,1);
plot(t, V_m, 'r', 'LineWidth', 1.5);
xlabel('Time (s)');ylabel('Velocity (m/s)');title('Missile Velocity over Time');
subplot(4,3,2);
plot(t, rad2deg(theta), 'r', 'LineWidth', 1.5);
xlabel('Time (s)');ylabel('\theta (deg)');title('Theta Angle');
subplot(4,3,3);
plot(t, rad2deg(phi_V), 'r', 'LineWidth', 1.5);
xlabel('Time (s)');ylabel('\phi_V (deg)');title('Phi_V Angle');

subplot(4,3,4);
plot(t, omega_x, 'g', 'LineWidth', 1.5);
xlabel('Time (s)');ylabel('\omega_x (rad/s)');title('Angular Velocity - \omega_x');
subplot(4,3,5);
plot(t, omega_y, 'g', 'LineWidth', 1.5);
xlabel('Time (s)');ylabel('\omega_y (rad/s)');title('Angular Velocity - \omega_y');
subplot(4,3,6);
plot(t, omega_z, 'g', 'LineWidth', 1.5);
xlabel('Time (s)');ylabel('\omega_z (rad/s)');title('Angular Velocity - \omega_z');

subplot(4,3,7);
plot(t, rad2deg(nu), 'b', 'LineWidth', 1.5);
xlabel('Time (s)');ylabel('\nu (deg)');title('Nu Angle');
subplot(4,3,8);
plot(t, rad2deg(phi), 'b', 'LineWidth', 1.5);
xlabel('Time (s)');ylabel('\phi (deg)');title('Phi Angle');
subplot(4,3,9);
plot(t, rad2deg(gama), 'b', 'LineWidth', 1.5);
xlabel('Time (s)');ylabel('\gamma (deg)');title('Gama Angle');

subplot(4,3,10);
plot(t, missile.recode.n_x2, 'r', 'LineWidth', 1.5);
xlabel('Time (s)');ylabel('n_x2');title('n_x2 over Time');
subplot(4,3,11);
plot(t, missile.recode.n_y2, 'r', 'LineWidth', 1.5);
xlabel('Time (s)');ylabel('n_y2');title('n_y2 over Time');
subplot(4,3,12);
plot(t, missile.recode.n_z2, 'r', 'LineWidth', 1.5);
xlabel('Time (s)');ylabel('n_z2');title('n_z2 over Time');

%% ode, event
function dy_dt = ode_wrap(t, y, missile, target)
    % ode45求解
    i = round((t - missile.t0) / missile.dt) + 1;

    missile_states = y(1:13);
    target_states = y(14:end); 

    delta_x = missile.recode.delta_y(i);
    delta_z = missile.recode.delta_z(i);

    dy_dt_missile = missile.Missile_Dynamics(t, missile_states, delta_x, delta_z);
    dy_dt_target = target.Target_Dynamics(target_states);
    dy_dt = [dy_dt_missile; dy_dt_target];
end

function [value, isterminal, direction] = event_wrap(t, y, missile)
    isterminal = 1; % 终止仿真
    direction = 0;
    value = 1;

    missile_pos = y(4:6); % 导弹位置
    target_pos = y(17:19); % 目标位置

    if missile.Hit_Ground(missile_pos) == 1
        value = 0; % 触发终止
    end
    if missile.Hit_Target(missile_pos, target_pos) == 1
        value = 0; % 触发终止
    end
end