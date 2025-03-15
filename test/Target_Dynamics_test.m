close all; clear; clc;

% y = [Vm, theta, phi_V, xm, ym, zm]
y0 = [10, deg2rad(0), deg2rad(0), 1000, 1000, 0];


% 定义微分方程
dynamics = @(t, y) Target_Dynamics(t, y, 'straight');

[t, y] = ode_EPC(0, 0.01, 5, y0, dynamics);

% 提取结果
V = y(1, :);
theta = y(2, :);
phi_V = y(3, :);
xt = y(4, :);
yt = y(5, :);
zt = y(6, :);

% 绘制结果
figure;
subplot(3, 1, 1);
plot(t, V);
xlabel('时间 (秒)');
ylabel('速度 (m/s)');
title('速度随时间变化');

subplot(3, 1, 2);
plot(t, theta);
xlabel('时间 (秒)');
ylabel('弹道倾角 (rad)');
title('弹道倾角随时间变化');

subplot(3, 1, 3);
plot(t, phi_V);
xlabel('时间 (秒)');
ylabel('弹道偏角 (rad)');
title('弹道偏角随时间变化');

figure;
plot3(xt, zt, yt);
xlabel('x (m)');
ylabel('z (m)');
zlabel('y (m)');
title('目标轨迹');
grid on;