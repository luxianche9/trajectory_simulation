clear;
clc;
close all;

%% 初值设置
% 导弹状态
V_m0 = 100;
theta_m0 = deg2rad(0);
phiV_m0 = deg2rad(0);
x_m0 = 0;
y_m0 = 0;
z_m0 = 0;
m0 = 1;
% 目标状态
V_t0 = 50;
theta_t0 = deg2rad(0);
phiV_t0 = deg2rad(0);
x_t0 = 1000;
y_t0 = 0;
z_t0 = 0;

y0 = [V_m0, theta_m0, phiV_m0, x_m0, y_m0, z_m0, m0 ...
      V_t0, theta_t0, phiV_t0, x_t0, y_t0, z_t0];

%% 仿真时间设置
t0 = 0;
dt = 0.01;
tf = 30;

%% 其他设置
% 目标飞行方式: 目标运动模式 ('circle', 'straight', 'stationary')
target_pattern = 'straight';
% 比例导引系数
N = 4;

%% 数值求解动态方程

f = @(t,y) simulation(t, y, target_pattern, N);

event = @(t, y) hit(t, y);

[t, y] = ode_EPC(t0, dt, tf, y0, f, event);

% 导弹
x_m = y(4, :);
y_m = y(5, :);
z_m = y(6, :);
% 目标
x_t = y(11, :);
y_t = y(12, :);
z_t = y(13, :);

%% 结果可视化

figure;
hold on;
plot3(x_m, z_m, y_m, 'b-', "DisplayName", 'missel');
if strcmp(target_pattern, 'stationary')
    plot3(x_t0, z_t0, y_t0, 'ro', "DisplayName", 'target');
else
    plot3(x_t, z_t, y_t, 'r--',"DisplayName", 'target');
end
xlabel('xm (m)');
ylabel('zm (m)');
zlabel('ym (m)');
legend;
view(3);
grid on;
title('导弹与目标轨迹')
hold off;

%% 仿真系统动态方程

function dydt = simulation(t, y, target_pattern, N)
    % 导弹
    V_m = y(1);
    theta_m = y(2);
    phiV_m = y(3);
    % 目标
    y2 = y(8:13);

    [dtheta_dt, dphiV_dt] = PN_guidance(y, N);

    dxm_dt = V_m * cos(theta_m) * cos(phiV_m);
    dym_dt = V_m * sin(theta_m);
    dzm_dt = - V_m * cos(theta_m) * sin(phiV_m);
    
    dy1dt = [0;
            dtheta_dt;
            dphiV_dt;
            dxm_dt;
            dym_dt;
            dzm_dt;
            0];

    dy2dt = Target_Dynamics(t, y2, target_pattern);

    dydt = [dy1dt; dy2dt];
end