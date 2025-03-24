clear; clc; close all;

%% 仿真时间设置
global dt;
t0 = 0;
dt = 0.01;
tf = 50;

%% 仿真参数设置
% 导弹
Vm0 = 20;
theta0 = deg2rad(18);
wz0 = deg2rad(0);
xm0 = 0;
ym0 = 20;
nu0 = deg2rad(18);
m0 = 52.38;
% 
Vtx0 = 15;
Vtz0 = 5;
xt0 = 

y0 = [Vm0, theta0, wz0, xm0, ym0, nu0, m0];

%% 数值求解动态方程
event = @(t, y) hit_ground(t, y);

ode = @(t, y) simulation(t, y, 0);

[t, y] = ode_EPC(t0, dt, tf, y0, ode, event);

%% 求解结果提取
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
    % 导引头环节
    phi_s = y(15);
    theta_s = y(16);
    % 非动态量存储
    global dt;
    i = round(t / dt) + 1;
    global a_mag_list;
    global phi_s_c_list;
    global theta_s_c_list;

    %% 导引头环节
    % 计算相对距离矢量
    r_rel = y(11:13)' - y(4:6)';
    % 计算导弹和目标的速度矢量
    V_t_vec = Euler2Vec(V_t, theta_t, phiV_t);
    V_m_vec = Euler2Vec(V_m, theta_m, phiV_m);
    V_rel = V_t_vec - V_m_vec;
    % 指令实际视线偏角的视线倾角
    phi_s_c = asin(- r_rel(3) / norm([r_rel(1) r_rel(3)]));
    theta_s_c = asin(r_rel(2) / norm(r_rel));
    theta_s_c_list(i) = theta_s_c;
    phi_s_c_list(i) = phi_s_c;
    
    % 导引头一阶环节
    T = 0.2;
    dphi_s = First_Order_Process(t, phi_s, phi_s_c, T);
    dtheta_s = First_Order_Process(t, theta_s, theta_s_c, T);

    L_is = eul2rotm([0, phi_s, theta_s], 'XYZ');
    r_rel_o = norm(r_rel) * L_is * [1; 0; 0];

    % 比例导引法
    omega = cross(r_rel_o, V_rel) / norm(r_rel_o)^2;
    a = - N * norm(V_rel) * cross(V_m_vec, omega) / norm(V_m_vec);

    % 将惯性坐标系中的加速度转化到速度坐标系
    L_vi = eul2rotm([0, phiV_m, theta_m], 'XYZ')';
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