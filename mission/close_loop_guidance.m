clear; clc; close all;

% 常数
R_Earth = 6371.4; % 地球半径 (km)
mu = 398600; % 地球引力常数 (km^3/s^2)
T = 20000; % 发动机推力 (N)
I_sp = 2842; % 发动机比冲 (s)
t_burn = 30; % 发动机工作时间 (s)

% 地心惯性坐标系下, 导弹位置和速度矢量
m_k0 = 500;
r_k0 = [1164; -5507; 3258];
v_k0 = [1.337; 0.743; 1.029];

% 地心坐标系下, 目标位置矢量
r_t = [1.289983766295003e3; -5.359931448185626e3; 3.194024762781843e3];

% ODE45求解微分方程
t0 = 0;
dt = 0.1;
tf = 100;

y0 = [r_k0; v_k0];
options = odeset('Events', @(t, y) event_wrap(t, y, R_Earth));
[t, y] = ode45(@(t, y) ode(t, y, mu, I_sp, t_burn), [t0 tf], y0, options);

% 提取结果, 绘图


figure;
hold on;

% [x_sphere, y_sphere, z_sphere] = sphere(50);
% x_sphere = x_sphere * R_Earth;
% y_sphere = y_sphere * R_Earth;
% z_sphere = z_sphere * R_Earth;
% earth_texture = imread('data/earth_texture.jpg');
% surf(x_sphere, y_sphere, z_sphere, ...
%     'FaceColor', 'texturemap', ...
%     'CData', earth_texture, ...
%     'EdgeColor', 'none');
plot3(y(:,1), y(:,2), y(:,3), 'b');
plot3(r_t(1), r_t(2), r_t(3), 'ro');

xlabel('X (km)'); ylabel('Y (km)'); zlabel('Z (km)');
title('火箭轨迹');
legend('地球', '火箭轨迹', '目标');
axis equal;grid on;view(3);

% 火箭微分方程组
function dydt = ode(t, y, mu, I_sp, t_burn)
    r = y(1:3);
    v = y(4:6);
    r_norm = norm(r);
    
    % 重力加速度
    g = -mu / r_norm^3 * r;
    
    % 计算推力加速度
    if t < t_burn
        Thrust = Rocket_Guidance(t, y);
        a_thrust = Thrust / I_sp;
    else
        a_thrust = [0; 0; 0];
    end
    
    % 总加速度
    a = g + a_thrust;
    dydt = [v; a];
end

% 火箭制导（无需编写此函数）
function Thrust = Rocket_Guidance(t, y)
    Thrust = [0; 0; 0]; % 仅作占位符
end

% 微分方程求解终止事件: 火箭与地球接触
function [value, isterminal, direction] = event_wrap(~, y, R_Earth)
    r = y(1:3);
    value = norm(r) - R_Earth;
    isterminal = 1;
    direction = -1;
end
