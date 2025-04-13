clear; clc; close all;

%% 测试轨道
a = 5900;       % 半长轴 (km)
e = 0.3;        % 偏心率
i = deg2rad(0);         % 倾角 (deg)
RAAN = deg2rad(0);      % 升交点赤经 (deg)
omega = deg2rad(-90);    % 近地点幅角 (deg)
r_eci = elements2orbit(a, e, i, RAAN, omega);

%% 测试轨道点
theta_M = deg2rad(140);
theta_T = deg2rad(200);
[r_M, v_M, gamma_M, lambda_M, phi_M] = orbital_state(a, e, i, RAAN, omega, theta_M);
[r_T, v_T, gamma_T, ~, ~] = orbital_state(a, e, i, RAAN, omega, theta_T);
fprintf("预设:\nV_M: %.2f km/s gama_M: %.2f deg \nlambda_M: %.2f deg phi_M: %.2f deg \n", ...
        norm(v_M), rad2deg(gamma_M), rad2deg(lambda_M), rad2deg(phi_M));

%% 测试计算需要速度
rocket = Rocket(0,0.1,10);
V_R = rocket.RocketGuidance(r_M, r_T, gamma_T);

fprintf("V_M预设:\n")
disp(v_M);
fprintf("V_R解算:\n")
disp(V_R);


%% 绘图
figure; hold on;
% 地球
plot_earth();
% 轨道
plot_orbit(r_eci);
% 轨道失径
quiver3(0,0,0,r_M(1)./ 0.9, r_M(2)./ 0.9, r_M(3)./ 0.9, 'r--', 'LineWidth', 2, 'DisplayName', '导弹失径');
quiver3(0,0,0,r_T(1)./ 0.9, r_T(2)./ 0.9, r_T(3)./ 0.9, 'b--', 'LineWidth', 2, 'DisplayName', '目标失径');
% 速度矢量
quiver3(r_M(1), r_M(2), r_M(3), ... 
        v_M(1) * 200, v_M(2) * 200, v_M(3) * 200, ...
        'r-', 'LineWidth', 2, 'DisplayName', '导弹速度矢量');
quiver3(r_T(1), r_T(2), r_T(3), ...
        v_T(1) * 200, v_T(2) * 200, v_T(3) * 200, ...
        'b-', 'LineWidth', 2, 'DisplayName', '目标速度矢量');

xlabel('X (km)'); ylabel('Y (km)'); zlabel('Z (km)');
title('Satellite Orbit in ECI Frame');
hold off; grid on; axis equal; view(3); legend("show");

function r_eci = elements2orbit(a, e, i, RAAN, omega)
    % 根据六要素计算轨道

    % 参数化真近点角范围（0到360度）
    theta = linspace(0, 2*pi, 500);

    % 计算在轨道平面内的轨道点 (PQW坐标系)
    r_pqw = (a * (1 - e^2)) ./ (1 + e * cos(theta));  % 长度为 1x500
    x_pqw = r_pqw .* cos(theta);
    y_pqw = r_pqw .* sin(theta);
    z_pqw = zeros(size(theta));  % 平面轨道 z = 0

    % 合并轨道点
    r_orbit_pqw = [x_pqw; y_pqw; z_pqw];

    % 旋转矩阵：从PQW变换到ECI
    R3_Omega = [cos(RAAN) -sin(RAAN) 0;
                sin(RAAN)  cos(RAAN) 0;
                     0          0    1];
    R1_i = [1      0           0;
            0 cos(i) -sin(i);
            0 sin(i)  cos(i)];
    R3_omega = [cos(omega) -sin(omega) 0;
                sin(omega)  cos(omega) 0;
                     0           0     1];

    % 总旋转
    Q_pqw2eci = R3_Omega * R1_i * R3_omega;

    % 变换到ECI坐标系
    r_eci = Q_pqw2eci * r_orbit_pqw;
end

function [r_eci, v_eci, gamma, lambda, phi] = orbital_state(a, e, i, RAAN, omega, nu)
    % 根据真近点角, 计算某一点失径和速度倾角
    % 常量
    mu = 398600.4418;  % 地球标准引力参数 (km^3/s^2)
    % 轨道参数
    p = a * (1 - e^2);
    r = p / (1 + e * cos(nu));

    % PQW 坐标系下的位置和速度矢量
    r_pqw = [r * cos(nu); r * sin(nu); 0];
    v_pqw = sqrt(mu / p) * [-sin(nu); e + cos(nu); 0];

    % 旋转矩阵 PQW -> ECI
    R3_Omega = [cos(RAAN), -sin(RAAN), 0;
                sin(RAAN),  cos(RAAN), 0;
                0        ,  0        , 1];
    R1_i = [1,      0,       0;
            0, cos(i), -sin(i);
            0, sin(i),  cos(i)];
    R3_omega = [cos(omega), -sin(omega), 0;
                sin(omega),  cos(omega), 0;
                0         ,  0         , 1];

    Q = R3_Omega * R1_i * R3_omega;

    % 转换到 ECI 坐标系
    r_eci = Q * r_pqw;
    v_eci = Q * v_pqw;

    % 飞行路径角（速度倾角）
    gamma = atan2(e * sin(nu), 1 + e * cos(nu));

    % 经纬度计算
    r_norm = norm(r_eci);
    phi = asin(r_eci(3) / r_norm);  % 纬度
    lambda = atan2(r_eci(2), r_eci(1));  % 经度 (考虑地球自转)

    % 将经度限制在 -pi 到 pi 范围
    lambda = mod(lambda + pi, 2*pi) - pi;
end

function plot_earth()
    [x, y, z] = sphere(100);
    R_earth = 6378.137;  % 地球半径 (km)
    x = R_earth * x;
    y = R_earth * y;
    z = R_earth * z;
    earth_img = imread('data/earth_texture.jpg');
    surf(x, y, z, ...
        'FaceColor', 'texturemap', ...
        'CData', earth_img, ...
        'EdgeColor', 'none', ...
        'FaceAlpha', 0.5, ...
        'DisplayName', 'Earth');
end

function plot_orbit(r_eci)
    R_earth = 6371.4; % 地球半径 (km)
    r_norm = vecnorm(r_eci);        % 每点距离
    is_inside = r_norm < R_earth;   % 是否在地球内部
    change_idx = [1, find(diff(is_inside) ~= 0) + 1, size(r_eci, 2)+1];
    plotted_inside = false;
    plotted_outside = false;
    for k = 1:length(change_idx)-1
        idx = change_idx(k):change_idx(k+1)-1;
        seg = r_eci(:, idx);
        if is_inside(idx(1))
            if ~plotted_inside
                plot3(seg(1,:), seg(2,:), seg(3,:), ...
                    'k--', 'LineWidth', 1.5, 'DisplayName', 'Orbit (Inside Earth)');
                plotted_inside = true;
            else
                plot3(seg(1,:), seg(2,:), seg(3,:), 'k--', 'LineWidth', 1.5);
            end
        else
            if ~plotted_outside
                plot3(seg(1,:), seg(2,:), seg(3,:), ...
                    'k-', 'LineWidth', 1.5, 'DisplayName', 'Orbit (Outside Earth)');
                plotted_outside = true;
            else
                plot3(seg(1,:), seg(2,:), seg(3,:), 'k-', 'LineWidth', 1.5);
            end
        end
    end
end