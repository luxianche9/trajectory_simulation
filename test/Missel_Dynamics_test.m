close all; clear; clc;
% 测试 Missel_Dynamics 函数并求解微分方程
Vm0 = 1;
theta0 = deg2rad(10);
phi_V0 = deg2rad(0);
xm0 = 0;
ym0 = 0;
zm0 = 0;
m0 = 1; % kg
% y = [Vm, theta, phi_V, xm, ym, zm, m]
y0 = [Vm0, theta0, phi_V0, xm0, ym0, zm0, m0];

% 目标角速度
dtheta_dt_target = deg2rad(1); % 弹道倾角变化率 (rad/s)
dphi_V_dt_target = 0; % 弹道偏角变化率 (rad/s)

% 定义微分方程
dynamics = @(t, y) Missel_Dynamics(t, y, dtheta_dt_target, dphi_V_dt_target);

[t, y] = ode_EPC(0, 0.01, 5, y0, dynamics);

% 提取结果
Vm = y(1, :);
theta = y(2, :);
phi_V = y(3, :);
xm = y(4, :);
ym = y(5, :);
zm = y(6, :);
m = y(7, :);

% 绘制结果
figure;
subplot(2, 2, 1);
plot(t, Vm);
xlabel('时间 (秒)');
ylabel('速度 (m/s)');
title('速度随时间变化');

subplot(2, 2, 2);
plot(t, theta);
xlabel('时间 (秒)');
ylabel('弹道倾角 (rad)');
title('弹道倾角随时间变化');

subplot(2, 2, 3);
plot(t, phi_V);
xlabel('时间 (秒)');
ylabel('弹道偏角 (rad)');
title('弹道偏角随时间变化');

subplot(2, 2, 4);
plot(t, m);
xlabel('时间 (秒)');
ylabel('质量(kg)');
title('质量随时间变化');

figure;
plot3(xm, zm, ym);
xlabel('xm (m)');
ylabel('zm (m)');
zlabel('ym (m)');
title('导弹轨迹');
grid on;