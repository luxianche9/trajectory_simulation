function dydt = Missel_Dynamics(t, y, dtheta_dt_target, dphi_V_dt_target)
    % 导弹动力学方程组(瞬时平衡假设)
    % y = [Vm, theta, phi_V, xm, ym, zm, m]
    Vm = y(1);
    theta = y(2); % 弹道倾角(rad)
    phi_V = y(3); % 弹道偏角(rad)
    zm = y(6);
    m = y(7);

    % 重力, 推力
    g = 9.8;
    G = m * g;
    m_rate = M_rate(t);
    P = Thrust(t);
    % 气动力
    rho = Air_Density(zm);
    S_ref = 0.0012566;
    q = 1/2 * rho * Vm ^ 2;
    % 轴对称导弹
    Cy_alpha = 10.4;
    Cy_dz = 1.51;
    Cz_beta = 10.4;
    Cz_dy = 1.51;
    mz_alpha = -6.07;
    mz_dz = -0.425;
    my_beta = -6.07;
    my_dy = -0.425;

    % 瞬时平衡假设
    alpha = (m * Vm * dtheta_dt_target + G * cos(theta)) ...
          / (P + Cy_alpha * q * S_ref);
    beta = (m * Vm * dphi_V_dt_target * cos(theta)) ...
          / (P + Cz_beta * q * S_ref);

    % 攻角, 侧滑角大小限制
    limit = deg2rad(10);
    alpha = max(- limit, min(limit, alpha));
    beta = max(- limit, min(limit, beta));

    delta_z = - mz_alpha / mz_dz * alpha;
    delta_y = - my_beta / my_dy * beta;

    fprintf('dtheta: %.2f alpha: %.2f (rad)\n', dtheta_dt_target, rad2deg(alpha));

    Cy = Cy_alpha * alpha + Cy_dz * delta_z;
    Cz = Cz_beta * beta + Cz_dy * delta_y;
    Cx = 0.437 + 7.01 * alpha * delta_z + 17.3 * alpha ^ 2 + 2.41 * delta_z ^ 2 ...
               + 7.01 * beta * delta_y + 17.3 * beta ^ 2 + 2.41 * delta_y ^ 2;
    
    Y = Cy * q * S_ref;
    X = Cx * q * S_ref;
    Z = Cz * q * S_ref;

    % 导弹动力学方程组
    dV_dt = (P * cos(alpha) * cos(beta) - X - G * sin(theta)) ...
          / m;
    dtheta_dt = (P * sin(alpha) + Y - G * cos(theta)) ...
              / (m * Vm);
    dphi_V_dt = (P * cos(alpha) * sin(beta) - Z) ...
              / (m * Vm * cos(theta));
    dxm_dt = Vm * cos(theta) * cos(phi_V);
    dym_dt = Vm * sin(theta);
    dzm_dt = - Vm * cos(theta) * sin(phi_V);
    dm_dt = -1 * m_rate;

    dydt = [dV_dt;
        dtheta_dt;
        dphi_V_dt;
        dxm_dt;
        dym_dt;
        dzm_dt;
        dm_dt];
end