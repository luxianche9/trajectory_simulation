clear;
clc;
close all;

%% 初值设置
% y = [V_m, theta_m, phiV_m, x_m, y_m, z_m, m, 
%      V_t, theta_t, phiV_t, x_t, y_t, z_t]
% 导弹状态
V_m0 = 150;
theta_m0 = deg2rad(0);
phiV_m0 = deg2rad(0);
x_m0 = 0;
y_m0 = 0;
z_m0 = 0;
m0 = 1; % kg
% 目标状态
V_t0 = 50;
theta_t0 = deg2rad(0);
phiV_t0 = deg2rad(0);
x_t0 = 1000;
y_t0 = 300;
z_t0 = 0;


y0 = [V_m0, theta_m0, phiV_m0, x_m0, y_m0, z_m0, m0, ...
     V_t0, theta_t0, phiV_t0, x_t0, y_t0, z_t0];

%% 仿真时间设置
t0 = 0;
dt = 0.01;
tf = 30;

%% 其他设置
% 目标飞行方式: 目标运动模式 ('circle', 'straight', 'stationary')
target_pattern = 'circle';
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

pos_m = [x_m; y_m; z_m];
pos_t = [x_t; y_t; z_t];
L = [1 0 0;
    0 0 -1;
    0 1 0];

pos_m = L * pos_m;
pos_t = L * pos_t;
plot3(pos_m(1, :), pos_m(2,:), pos_m(3,:), 'b-', "DisplayName", 'missel');
plot3(pos_t(1,:), pos_t(2,:), pos_t(3,:), 'r--',"DisplayName", 'target');


xlabel('xm (m)');
ylabel('zm (m)');
zlabel('ym (m)');
legend;
view(3);
axis equal;
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

    a_v = PN_guidance(y, N);
    dtheta_dt = (a_v(2) - 9.8 * cos(theta_m)) / V_m;
    dphiV_dt = - a_v(3) / (V_m * cos(theta_m));

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