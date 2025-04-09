clear; clc; close all;

% 常数
global R_Earth;
global mu;
R_Earth = 6371.4; % 地球半径 (km)
mu = 398600; % 地球引力常数 (km^3/s^2)

% 地心惯性坐标系下, 导弹位置和速度矢量
m_k0 = 500;
r_k0 = [1164; -5507; 3258];
v_k0 = [1.337; 0.743; 1.029];

% 地心坐标系下, 目标位置矢量
r_p = [1.289983766295003e3;
      -5.359931448185626e3;
       3.194024762781843e3];
theta_p = deg2rad(-55.5302);

% ODE45求解微分方程
t0 = 0;
dt = 0.1;
tf = 30;

y0 = [r_k0; v_k0; m_k0];
% options = odeset('Events', @(t, y) event_wrap(t, y, R_Earth));
% [t, y] = ode45(@(t, y) ode(t, y, mu, I_sp, t_burn, r_p, theta_p, T), [t0 tf], y0, options);

event = @(t, y) event_wrap(t, y, R_Earth);

simulation = @(t, y) ode(t, y, mu, I_sp, t_burn, r_p, theta_p, T);

[t, y] = ode_EPC(t0, dt, tf, y0, simulation);

y = y';

% 提取结果, 绘图
figure;
hold on;
plot3(y(:,1), y(:,2), y(:,3), 'b');
plot3(r_p(1), r_p(2), r_p(3), 'ro');
xlabel('X (km)'); ylabel('Y (km)'); zlabel('Z (km)');
title('火箭轨迹');
legend('火箭轨迹', '目标');
axis equal;grid on;view(3);

% 火箭制导（无需编写此函数）
function u = Rocket_Guidance(y, r_p, theta_p)
    mu = 398600; % 地球引力常数 (km^3/s^2)
    r = y(1:3);
    v = y(4:6);
    r_norm = norm(r);
    r_p_norm = norm(r_p);
    
    % 计算射程角β_e
    cos_beta_e = dot(r, r_p) / (r_norm * r_p_norm);
    beta_e = acos(cos_beta_e);
    
    % 计算半通径p
    denominator = 1 - (r_norm/r_p_norm) * (cos_beta_e + tan(theta_p)*sin(beta_e));
    p = (r_norm * (1 - cos_beta_e)) / denominator;
    
    % 计算β_p和θ_l0
    beta_p = atan(tan(theta_p) / (1 - r_p_norm/p));
    theta_l0 = atan((1 - r_norm/p) * tan(beta_p - beta_e));
    
    % 需要速度大小
    v_r = sqrt(mu * p) / (r_norm * cos(theta_l0));
    
    % 计算经纬度
    lambda0 = atan2(r(2), r(1));
    lambda_p = atan2(r_p(2), r_p(1));
    phi0 = asin(r(3)/r_norm);
    phi_p = asin(r_p(3)/r_p_norm);
    
    % 计算方位角α_e
    delta_lambda = lambda_p - lambda0;
    sin_alpha_e = sin(abs(delta_lambda)) * cos(phi_p) / sin(beta_e);
    cos_alpha_e = (sin(phi_p) - cos_beta_e*sin(phi0)) / (cos(phi0)*sin(beta_e));
    
    % 确定α_e象限
    if abs(sin_alpha_e) <= abs(cos_alpha_e)
        if cos_alpha_e >= 0
            alpha_e = asin(sin_alpha_e);
        else
            alpha_e = pi*sign(sin_alpha_e) - asin(sin_alpha_e);
        end
    else
        alpha_e = sign(sin_alpha_e)*acos(cos_alpha_e);
    end
    
    % 分解到当地坐标系
    v_rx1 = v_r * sin(theta_l0);
    v_ry1 = v_r * cos(theta_l0) * sin(alpha_e);
    v_rz1 = v_r * cos(theta_l0) * cos(alpha_e);
    
    % 转换到惯性系
    C_z = [cos(-lambda0), -sin(-lambda0), 0;
           sin(-lambda0),  cos(-lambda0), 0;
           0, 0, 1];
    C_y = [cos(phi0), 0, -sin(phi0);
           0, 1, 0;
           sin(phi0), 0, cos(phi0)];
    v_r_inertial = C_z * C_y * [v_rx1; v_ry1; v_rz1];
    
    % 计算推力方向
    v_g = v_r_inertial - v;
    v_g_norm = norm(v_g);
    u = v_g / v_g_norm; % 单位矢量
end

% 微分方程求解终止事件: 火箭与地球接触
function value = event_wrap(~, y, R_Earth)
    r = y(1:3);
    value = norm(r) - R_Earth;
    isterminal = 1;
    direction = -1;
end
