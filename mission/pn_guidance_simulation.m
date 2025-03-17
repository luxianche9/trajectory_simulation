clear;
clc;
close all;

%% 仿真时间设置
t0 = 0;
global dt;
dt = 0.01;
tf = 20;

%% 初值设置
% y = [V_m, theta_m, phiV_m, x_m, y_m, z_m, m, 
%      V_t, theta_t, phiV_t, x_t, y_t, z_t]
% 导弹状态
V_m0 = 10;
theta_m0 = deg2rad(20);
phiV_m0 = deg2rad(0);
x_m0 = 0;
y_m0 = 0;
z_m0 = 0;
m0 = 1; % kg
% 目标状态
V_t0 = 100;
theta_t0 = deg2rad(0);
phiV_t0 = deg2rad(0);
x_t0 = 500;
y_t0 = 300;
z_t0 = 0;
% 目标飞行方式: 目标运动模式 ('circle', 'straight', 'stationary', 'random')
target_pattern = 'straight';
% 比例导引系数
N = 6;
% 一阶环节自动驾驶仪输出
a_control0 = 0;
% 导引头环节
phi_s0 = 0;
theta_s0 = 0;

y0 = [V_m0, theta_m0, phiV_m0, x_m0, y_m0, z_m0, m0, ...
     V_t0, theta_t0, phiV_t0, x_t0, y_t0, z_t0, ...
     a_control0, ...
     phi_s0, theta_s0];

% 非动态量存储
% 索引: i = round(t / dt) + 1;
list_length = length(t0:dt:tf) - 1;
% 指令加速度
global a_mag_list;
global theta_s_c_list;
global phi_s_c_list;
a_mag_list = zeros(1, list_length);
theta_s_c_list = zeros(1, list_length);
phi_s_c_list = zeros(1, list_length);

%% 数值求解动态方程

f = @(t,y) simulation(t, y, target_pattern, N);

event = @(t, y) hit(t, y);

[t, y] = ode_EPC(t0, dt, tf, y0, f, event);

% 求解结果提取
% 导弹
V_m = y(1, :);
theta_m = y(2, :);% 弹道倾角
phiV_m = y(3, :);% 弹道偏角
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
% 自动驾驶仪
a_control = y(14, :);
phi_s = y(15, :);
theta_s = y(16, :);

%% 结果可视化

% 绘制结果
figure;
subplot(3, 3, 1);
plot(t, V_m);
xlabel('时间 (秒)');
ylabel('速度 (m/s)');
title('速度随时间变化');

subplot(3, 3, 2);
plot(t, m);
xlabel('时间 (秒)');
ylabel('质量(kg)');
title('质量随时间变化');

if length(a_mag_list) > length(t)
    a_mag_list = a_mag_list(1:length(t));
end
subplot(3, 3, 3);
hold on;
plot(t, a_mag_list, 'r--', "DisplayName", 'input');
plot(t, a_control, 'b-', 'DisplayName', 'output');
xlabel('时间 (秒)');
ylabel('加速度大小(m/s^2)');
title('自动驾驶仪输出对比输入');
legend;
hold off;

if length(phi_s_c_list) > length(phi_s)
    phi_s_c_list = phi_s_c_list(1:length(phi_s));
    theta_s_c_list = theta_s_c_list(1:length(phi_s));
end
subplot(3,3,4);
hold on;
plot(t, phi_s, 'b-', DisplayName='phi_s');
plot(t, phi_s_c_list, 'r--', DisplayName='phi_s_c');
title('phi_s');
legend;
hold off;

subplot(3,3,5);
hold on;
plot(t, theta_s, 'b-', DisplayName='theta_s');
plot(t, theta_s_c_list, 'r--', DisplayName='theta_s_c');
title('theta_s');
legend;
hold off;

subplot(3,3,[7 8 9])
hold on;
pos_m = [x_m; y_m; z_m];
pos_t = [x_t; y_t; z_t];
L = [1 0 0;
    0 0 -1;
    0 1 0];
pos_m = L * pos_m;
pos_t = L * pos_t;
plot3(pos_m(1, :), pos_m(2,:), pos_m(3,:), 'b-', "DisplayName", '导弹');
plot3(pos_t(1,:), pos_t(2,:), pos_t(3,:), 'r--',"DisplayName", '目标');
plot3(pos_m(1,1), pos_m(2,1), pos_m(3,1), 'bo', 'DisplayName', "导弹发射点");
plot3(pos_t(1,1), pos_t(2,1), pos_t(3,1), 'ro', 'DisplayName', "目标起始点");
plot3(pos_m(1,end), pos_m(2,end), pos_m(3, end), 'mx', 'DisplayName', "拦截点")
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
    % 导弹状态
    y1 = y(1:7);
    V_m = y(1);
    theta_m = y(2);
    phiV_m = y(3);
    x_m = y(4);
    y_m = y(5);
    z_m = y(6);
    % 目标状态
    y2 = y(8:13);
    V_t = y(8);
    theta_t = y(9);
    phiV_t = y(10);
    x_t = y(11);
    y_t = y(12);
    z_t = y(13);
    % 自动驾驶仪输出
    a_control = y(14);
    phi_s = y(15);
    theta_s = y(16);

    % 中间数据存储
    global dt;
    i = round(t / dt) + 1;
    global a_mag_list;
    global phi_s_c_list;
    global theta_s_c_list;


    % fprintf('t: %.2f i: %d a_c: %.2f \n', t, i, a_control);
    
    % 计算相对距离矢量
    r_rel = [x_t - x_m;
             y_t - y_m;
             z_t - z_m];
    % 计算导弹和目标的速度矢量
    V_m_vec = [V_m .* cos(theta_m) .* cos(phiV_m);
            V_m .* sin(theta_m);
            - V_m .* cos(theta_m) .* sin(phiV_m)];
    V_t_vec = [V_t .* cos(theta_t) .* cos(phiV_t);
            V_t .* sin(theta_t);
            - V_t .* cos(theta_m) .* sin(phiV_m)];
    % 计算相对速度矢量
    V_rel = V_t_vec - V_m_vec;

    phi_s_c = asin(- r_rel(3) / norm([r_rel(1) r_rel(3)]));
    theta_s_c = asin(r_rel(2) / norm(r_rel));
    theta_s_c_list(i) = theta_s_c;
    phi_s_c_list(i) = phi_s_c;
    
    % 导引头环节
    T = 0.2;
    dphi_s = First_Order_Process(t, phi_s, phi_s_c, T);
    dtheta_s = First_Order_Process(t, theta_s, theta_s_c, T);

    L_is = eul2rotm([0, phi_s, theta_s], 'XYZ');
    r_rel_o = norm(r_rel) * L_is * [1; 0; 0];
    disp(r_rel_o - r_rel);

    % 比例导引法
    omega = cross(r_rel_o, V_rel) / norm(r_rel_o)^2;
    a = - N * norm(V_rel) * cross(V_m_vec, omega) / norm(V_m_vec);

    % L = eul2rotm([0, phiV_m, theta_m], 'XYZ')';
    L_vi = [cos(theta_m)*cos(phiV_m), sin(theta_m), -cos(theta_m)*sin(phiV_m);
           -sin(theta_m)*cos(phiV_m), cos(theta_m),  sin(theta_m)*sin(phiV_m);
            sin(phiV_m),              0,             cos(phiV_m)];
    a_v = L_vi * a;
    a_mag = norm(a_v);
    a_dir = a_v/a_mag;

    % 加速度饱和
    a_max = 30 * 9.8;
    a_mag = max(- a_max, min(a_max, a_mag));
    a_mag_list(i) = a_mag;

    % 自动驾驶仪
    T = 0.1;
    da_control = First_Order_Process(t, a_control, a_mag, T);
    % 加速度转化到弹道加速度特性
    a_v = a_control * a_dir;
    dtheta_dt = (a_v(2) - 9.8 * cos(theta_m)) / V_m;
    dphiV_dt = - a_v(3) / (V_m * cos(theta_m));

    % 导弹动力学方程, 目标动力学方程
    dy1dt = Missel_Dynamics(t, y1, dtheta_dt, dphiV_dt);
    dy2dt = Target_Dynamics(t, y2, target_pattern);

    dydt = [dy1dt; dy2dt; da_control; dphi_s; dtheta_s];
end