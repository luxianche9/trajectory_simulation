clear; clc; close all;

%% 仿真时间设置
t0 = 0;
dt = 0.01;
tf = 100;

%% 仿真参数设置
% 导弹初始状态
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

y0 = [V0; theta0; phi_V0; xm0; ym0; zm0; omega_x0; omega_y0; omega_z0; nu0; phi0; gama0; m0; ...
    ];

%% 导弹初始化
missile = Missile(t0, dt, tf);

%% 数值求解动态方程

event =  @(t, y) missile.Hit_Ground(t, y);

ode = @(t, y) missile.Missile_Dynamics(t, y, 0, 0);

options = odeset('Events', event, 'RelTol', 1e-6, 'AbsTol', 1e-9);

[t, y] = ode45(ode, t0:dt:tf, y0, options);

%% 数据提取
% V, theta, phi_V, x, y, z, omega_x, omega_y, omega_z, nu, phi, gama, m
V = y(:, 1);
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

%% 数据可视化
figure; 
pos_m = [x_m'; y_m'; z_m'];
L = [1 0 0;
    0 0 -1;
    0 1 0];
pos_m = L * pos_m;
plot3(pos_m(1, :), pos_m(2,:), pos_m(3,:), 'b-', "DisplayName", 'missel');
xlabel('xm (m)'); ylabel('zm (m)'); zlabel('ym (m)'); title('Missile Trajectory');
grid on; view(3); legend('show');

figure;
subplot(3,3,1);
plot(t, V, 'r', 'LineWidth', 1.5);
xlabel('Time (s)');ylabel('Velocity (m/s)');title('Missile Velocity over Time');
subplot(3,3,2);
plot(t, rad2deg(theta), 'r', 'LineWidth', 1.5);
xlabel('Time (s)');ylabel('\theta (deg)');title('Theta Angle');
subplot(3,3,3);
plot(t, rad2deg(phi_V), 'r', 'LineWidth', 1.5);
xlabel('Time (s)');ylabel('\phi_V (deg)');title('Phi_V Angle');

subplot(3,3,4);
plot(t, omega_x, 'g', 'LineWidth', 1.5);
xlabel('Time (s)');ylabel('\omega_x (rad/s)');title('Angular Velocity - \omega_x');
subplot(3,3,5);
plot(t, omega_y, 'g', 'LineWidth', 1.5);
xlabel('Time (s)');ylabel('\omega_y (rad/s)');title('Angular Velocity - \omega_y');
subplot(3,3,6);
plot(t, omega_z, 'g', 'LineWidth', 1.5);
xlabel('Time (s)');ylabel('\omega_z (rad/s)');title('Angular Velocity - \omega_z');

subplot(3,3,7);
plot(t, rad2deg(nu), 'b', 'LineWidth', 1.5);
xlabel('Time (s)');ylabel('\nu (deg)');title('Nu Angle');
subplot(3,3,8);
plot(t, rad2deg(phi), 'b', 'LineWidth', 1.5);
xlabel('Time (s)');ylabel('\phi (deg)');title('Phi Angle');
subplot(3,3,9);
plot(t, rad2deg(gama), 'b', 'LineWidth', 1.5);
xlabel('Time (s)');ylabel('\gamma (deg)');title('Gama Angle');
