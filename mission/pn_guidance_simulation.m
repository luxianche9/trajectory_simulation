clear;
clc;
close all;

%% 初值设置
% y = [V_m, theta_m, phiV_m, x_m, y_m, z_m, m, 
%      V_t, theta_t, phiV_t, x_t, y_t, z_t]
% 导弹状态
V_m0 = 1;
theta_m0 = deg2rad(10);
phiV_m0 = deg2rad(0);
x_m0 = 0;
y_m0 = 0;
z_m0 = 0;
m0 = 1; % kg
% 目标状态
V_t0 = 10;
theta_t0 = deg2rad(0);
phiV_t0 = deg2rad(0);
x_t0 = 500;
y_t0 = 500;
z_t0 = 0;
% 自动驾驶仪状态

% 导引头状态

y0 = [V_m0, theta_m0, phiV_m0, x_m0, y_m0, z_m0, m0, ...
     V_t0, theta_t0, phiV_t0, x_t0, y_t0, z_t0];

%% 仿真时间设置
t0 = 0;
dt = 0.01;
tf = 5;

%% 其他设置
% 目标飞行方式: 目标运动模式 ('circle', 'straight', 'stationary')
target_pattern = 'straight';

%% 数值求解动态方程

f = @(t,y) simulation(t, y, target_pattern);

[t, y] = ode_EPC(t0, dt, tf, y0, f);

% 导弹
V_m = y(1, :);
theta_m = y(2, :);
phiV_m = y(3, :);
x_m = y(4, :);
y_m = y(5, :);
z_m = y(6, :);
m = y(7, :);
% 目标
V_t = y(8, :);
theta_t = y(9, :);
phiV_t = y(10, :);
x_t = y(11, :);
y_t = y(12, :);
z_t = y(13, :);

%% 结果可视化

figure;
hold on;
plot3(x_m, z_m, y_m, 'b-', "DisplayName", 'missel');
plot3(x_t, z_t, y_t, 'r--',"DisplayName", 'target');
xlabel('xm (m)');
ylabel('zm (m)');
zlabel('ym (m)');
legend;
view(3);
grid on;
title('导弹与目标轨迹')
hold off;

%% 仿真系统动态方程

function dydt = simulation(t, y, target_pattern)
    y1 = y(1:7);
    y2 = y(8:13);

    dtheta = 0;
    dphiV = 0;

    dy1dt = Missel_Dynamics(t, y1, dtheta, dphiV);
    dy2dt = Target_Dynamics(t, y2, target_pattern);

    dydt = [dy1dt; dy2dt];
end