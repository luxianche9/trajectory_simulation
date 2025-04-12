function r_eci = elements2orbit(a, e, i_deg, RAAN_deg, omega_deg)
    % 将角度转为弧度
    i = deg2rad(i_deg);
    RAAN = deg2rad(RAAN_deg);
    omega = deg2rad(omega_deg);

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
