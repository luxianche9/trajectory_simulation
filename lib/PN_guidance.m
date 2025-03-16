function [dtheta_dt, dphiV_dt] = PN_guidance(y, N)
    % 导弹
    V_m = y(1);
    theta_m = y(2);
    phiV_m = y(3);
    x_m = y(4);
    y_m = y(5);
    z_m = y(6);
    % 目标
    V_t = y(8);
    theta_t = y(9);
    phiV_t = y(10);
    x_t = y(11);
    y_t = y(12);
    z_t = y(13);

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
    % 比例导引法

    omega = cross(r_rel, V_rel) / norm(r_rel)^2;

    % a = - N * norm(V_rel) * cross(r_rel, omega) / norm(r_rel);
    % a = N * cross(V_rel, omega);
    a = - N * norm(V_rel) * cross(V_m_vec, omega) / norm(V_m_vec);

    theta = theta_m;
    phiV = phiV_m;
    L = [cos(theta)*cos(phiV), sin(theta), -cos(theta)*sin(phiV);
        -sin(theta)*cos(phiV), cos(theta), sin(theta)*sin(phiV);
        sin(phiV), 0, cos(phiV)];
    % L = eul2rotm([0, phiV_m, theta_m], 'XYZ');

    a_v = L * a;

    dtheta_dt = (a_v(2) - 9.8 * cos(theta_m)) / V_m;
    % dtheta_dt = (a_v(2)) / V_m;
    dphiV_dt = - a_v(3) / (V_m * cos(theta_m));
end
