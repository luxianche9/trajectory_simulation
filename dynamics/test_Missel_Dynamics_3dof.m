close all; clear; clc;
% 测试 Missel_Dynamics_3dof 函数并求解微分方程
Vm0 = 20;
theta0 = deg2rad(18);
wz0 = deg2rad(0);
xm0 = 0;
ym0 = 20;
nu0 = deg2rad(18);
m0 = 52.38; % kg
y0 = [Vm0, theta0, wz0, xm0, ym0, nu0, m0];

% 定义微分方程

event = @(t, y) hit_ground(t, y);

dynamics = @(t, y) Missel_Dynamics_3dof(t, y, 0);

[t, y] = ode_EPC(0, 0.01, 50, y0, dynamics, event);

% 提取结果
Vm = y(1, :);
theta = y(2, :);
wz = y(3, :);
xm = y(4, :);
ym = y(5, :);
nu = y(6, :);
m = y(7, :);

% 绘制结果
figure;
subplot(2, 4, 1);
plot(t, Vm);
xlabel('时间 (秒)');
ylabel('速度 (m/s)');
title('速度随时间变化');

subplot(2, 4, 2);
plot(t, rad2deg(theta));
xlabel('时间 (秒)');
ylabel('弹道倾角 (deg)');
title('弹道倾角随时间变化');

subplot(2, 4, 5);
plot(t, rad2deg(nu));
xlabel('时间 (秒)');
ylabel('俯仰角 (deg)');
title('俯仰角随时间变化');

subplot(2, 4, 6);
plot(t, m);
xlabel('时间 (秒)');
ylabel('质量(kg)');
title('质量随时间变化');

subplot(2, 4, [3,4,7,8]);
hold on;
plot(xm0, ym0, 'bo', DisplayName='反坦克导弹发射点')
plot(xm, ym, 'b-',DisplayName='反坦克导弹轨迹')
xlabel('xm (m)');
ylabel('ym (m)');
title('导弹轨迹')
legend;