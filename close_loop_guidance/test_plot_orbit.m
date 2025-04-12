clear; clc; close all;

% ========= 轨道六要素 =========
a = 5900;       % 半长轴 (km)
e = 0.3;        % 偏心率
i = 55;         % 倾角 (deg)
RAAN = 94;      % 升交点赤经 (deg)
omega = -90;    % 近地点幅角 (deg)


% ========= 轨道坐标 =========
r_eci = elements2orbit(a, e, i, RAAN, omega);


% ========= 地球绘制准备 =========
[x, y, z] = sphere(100);
R_earth = 6378.137;  % 地球半径 (km)
x = R_earth * x;
y = R_earth * y;
z = R_earth * z;
earth_img = imread('data/earth_texture.jpg');

% ========= 开始绘图 =========
figure; hold on;

% ====== 地球 ======
h_earth = surf(x, y, z, ...
    'FaceColor', 'texturemap', ...
    'CData', earth_img, ...
    'EdgeColor', 'none', ...
    'FaceAlpha', 0.5, ...
    'DisplayName', 'Earth');

% ====== 轨道分段绘图 ======
r_norm = vecnorm(r_eci);        % 每点距离
is_inside = r_norm < R_earth;   % 是否在地球内部
change_idx = [1, find(diff(is_inside) ~= 0) + 1, size(r_eci, 2)+1];

% 用于避免图例重复
plotted_inside = false;
plotted_outside = false;

for k = 1:length(change_idx)-1
    idx = change_idx(k):change_idx(k+1)-1;
    seg = r_eci(:, idx);
    
    if is_inside(idx(1))
        if ~plotted_inside
            h_inside = plot3(seg(1,:), seg(2,:), seg(3,:), ...
                'k--', 'LineWidth', 1.5, 'DisplayName', 'Orbit (Inside Earth)');
            plotted_inside = true;
        else
            plot3(seg(1,:), seg(2,:), seg(3,:), 'k--', 'LineWidth', 1.5);
        end
    else
        if ~plotted_outside
            h_outside = plot3(seg(1,:), seg(2,:), seg(3,:), ...
                'k-', 'LineWidth', 1.5, 'DisplayName', 'Orbit (Outside Earth)');
            plotted_outside = true;
        else
            plot3(seg(1,:), seg(2,:), seg(3,:), 'k-', 'LineWidth', 1.5);
        end
    end
end

% ====== 指定轨道点 ======
theta = 0;      % 真近点角 (deg)
[r_vec, gamma_angle] = orbital_state(a, e, i, RAAN, omega, theta);
plot3(r_vec(1), r_vec(2), r_vec(3), ...
    'ro', 'MarkerSize', 6, 'MarkerFaceColor', 'r', ...
    'DisplayName', sprintf('Rocket Point (\\gamma = %.2f°)', gamma_angle));

theta = 90;      % 真近点角 (deg)
[r_vec, gamma_angle] = orbital_state(a, e, i, RAAN, omega, theta);
plot3(r_vec(1), r_vec(2), r_vec(3), ...
    'bo', 'MarkerSize', 6, 'MarkerFaceColor', 'b', ...
    'DisplayName', sprintf('Target Point (\\gamma = %.2f°)', gamma_angle));
    


% ====== 图例 & 设置 ======
legend('Location', 'best');
xlabel('X (km)'); ylabel('Y (km)'); zlabel('Z (km)');
title('Satellite Orbit in ECI Frame');
grid on; axis equal; view(3);
