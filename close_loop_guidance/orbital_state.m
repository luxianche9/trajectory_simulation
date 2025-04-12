function [r_eci, gamma_deg] = orbital_state(a, e, i_deg, RAAN_deg, omega_deg, nu_deg)
    % 常量
    mu = 398600.4418;  % 地球标准引力参数 (km^3/s^2)

    % 角度 -> 弧度
    i = deg2rad(i_deg);
    RAAN = deg2rad(RAAN_deg);
    omega = deg2rad(omega_deg);
    nu = deg2rad(nu_deg);

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
    gamma_deg = rad2deg(gamma);
end
