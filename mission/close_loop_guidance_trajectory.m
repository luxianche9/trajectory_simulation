clear; clc; close all;

%% 初始值
m_M0 = 500; % kg
r_M0 = [1164; -5507; 3258]; % km
v_M0 = [1.337; 0.743; 1.029]; % km/s
r_T = [1289.9838; -5359.9314; 3194.0248]; % km
gama_T = deg2rad(-55.5302);
y0 = [r_M0; v_M0; m_M0];
t0 = 0; dt = 0.005; tf = 500; % s

rocket = Rocket(t0, dt, tf);
ode = @(t, y) rocket.RocketDynamics(t, y, r_T, gama_T);
event = @(t, y) rocket.HitGround(t, y);
[t, y] = ode_EPC(t0, dt, tf, y0, ode, event);

r_M = y(1:3, 1:length(t)); % 导弹位置
v_M = y(3:6, 1:length(t)); % 导弹速度
P = rocket.recoder.P(1:3, 1:length(t)); % 推力方向
r_rel = rocket.recoder.r_rel(1:length(t)); % 相对位置
r_T = rocket.recoder.r_T(1:3, 1:length(t)); % 目标位置矢量
V_R = rocket.recoder.V_R(1:3, 1:length(t)); % 目标速度矢量

figure; hold on;
% 地球
[x, y, z] = sphere(100);
R_earth = 6378.137;  % 地球半径 (km)
x = R_earth * x;
y = R_earth * y;
z = R_earth * z;
earth_img = imread('data/earth_texture.jpg');
surf(x, y, z, ...
    'FaceColor', 'texturemap', ...
    'CData', earth_img, ...
    'EdgeColor', 'none', ...
    'FaceAlpha', 0.5, ...
    'DisplayName', 'Earth');
% 导弹轨迹
plot3(r_M(1, :), r_M(2, :), r_M(3, :), 'r-', 'LineWidth', 2, 'DisplayName', '导弹轨迹');
% 导弹起点终点
plot3(r_M(1,1), r_M(2,1), r_M(3,1), 'go', 'MarkerSize', 4, 'DisplayName', '导弹起点');
plot3(r_M(1,end), r_M(2,end), r_M(3,end), 'kx', 'MarkerSize', 4, 'DisplayName', '导弹落点');
% 目标轨迹
plot3(r_T(1, :), r_T(2, :), r_T(3, :), 'b-', 'LineWidth', 2, 'DisplayName', '目标轨迹');

axis equal; grid on; view(45, 30);
xlabel('x (km)', 'FontSize', 12); 
ylabel('y (km)', 'FontSize', 12); 
zlabel('z (km)', 'FontSize', 12);
title('导弹与目标三维轨迹', 'FontSize', 14, 'FontWeight', 'bold');
legend('show', 'Location', 'best');

figure;
plot(t, r_rel, 'b-', 'LineWidth', 2);
xlabel('时间 (s)', 'FontSize', 12);
ylabel('相对距离 (m)', 'FontSize', 12);
title('导弹与目标间相对距离', 'FontSize', 14, 'FontWeight', 'bold');
grid on;


figure; hold on;
plot(t, P(1, :), 'r', 'LineWidth', 2, 'DisplayName', '推力方向 X');
plot(t, P(2, :), 'g', 'LineWidth', 2, 'DisplayName', '推力方向 Y');
plot(t, P(3, :), 'b', 'LineWidth', 2, 'DisplayName', '推力方向 Z');
xlabel('时间 (s)', 'FontSize', 12);
ylabel('推力方向分量', 'FontSize', 12);
title('推力方向随时间变化', 'FontSize', 14, 'FontWeight', 'bold');
legend('show'); grid on;

% 计算模值
v_M_mag = vecnorm(v_M, 2, 1);
V_R_mag = vecnorm(V_R, 2, 1);

figure; 
% plot(t, v_M_mag, 'r-', 'LineWidth', 2, 'DisplayName', '导弹速度');
% hold on;
plot(t, V_R_mag, 'b--', 'LineWidth', 2, 'DisplayName', '需要速度');
xlabel('时间 (s)', 'FontSize', 12);
ylabel('需要速度大小 (km/s)', 'FontSize', 12);
title('需要速度大小变化', 'FontSize', 14, 'FontWeight', 'bold');
legend('show'); grid on;