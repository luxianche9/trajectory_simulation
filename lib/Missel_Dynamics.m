function dydt = Missel_Dynamics(t, y, dtheta_dt_target, dphi_V_dt_target)
    % 导弹动力学方程组(瞬时平衡假设)
    % y = [Vm, theta, phi_V, xm, ym, zm, m]
    Vm = y(1);
    theta = y(2); % 弹道倾角(rad)
    phi_V = y(3); % 弹道偏角(rad)
    zm = y(6);
    m = y(7);

    % 重力
    g = 9.8;
    G = m * g;
    % 推力
    m_rate = M_rate(t);
    P = Thrust(t);
    % 气动力
    rho = Air_Density(zm);
    S_ref = 0.0012566;
    q = 1/2 * rho * Vm ^ 2;

    % 气动力
    Cy_alpha = 10.4;
    Cz_beta = Cy_alpha;
    Cy_dz = 1.51;
    Cz_dy = Cy_dz;
    % 气动转矩
    mz_alpha = -6.07;
    mz_dz = -0.425;
    my_beta = mz_alpha;
    my_dy = mz_alpha;
    % 瞬时平衡假设
    dz_alpha = - mz_alpha / mz_dz;
    dy_beta = - my_beta / my_dy;
    % 统一升力系数
    Y_alpha = (Cy_alpha + Cy_dz * dz_alpha)* q * S_ref;
    Z_beta = - (Cz_beta + Cz_dy * dy_beta) * q * S_ref;

    % 根据加速度反推攻角和侧滑角
    alpha = (m * Vm * dtheta_dt_target + G * cos(theta)) ...
          / (P + Y_alpha);
    beta = (m * Vm * dphi_V_dt_target * cos(theta)) ...
          / (P - Z_beta);
    % 攻角, 侧滑角大小限制
    limit = deg2rad(20);
    alpha = max(- limit, min(limit, alpha));
    beta = max(- limit, min(limit, beta));
    % fprintf('dtheta: %.2f alpha: %.2f (deg)\n', dtheta_dt_target, rad2deg(alpha));
    % fprintf('dphiV: %.2f beta: %.2f (deg)\n', dphi_V_dt_target, rad2deg(beta));
    Y = Y_alpha * alpha;
    Z = Z_beta * beta;
	X = (0.437 + 17.3 * alpha ^ 2) * q * S_ref;

    % 导弹动力学方程组
    dV_dt = (P * cos(alpha) * cos(beta) - X - G * sin(theta)) ...
          / m;
    dtheta_dt = (P * sin(alpha) + Y - G * cos(theta)) ...
              / (m * Vm);
    dphi_V_dt = (- P * cos(alpha) * sin(beta) + Z) ...
              / (- m * Vm * cos(theta));
    dxm_dt = Vm * cos(theta) * cos(phi_V);
    dym_dt = Vm * sin(theta);
    dzm_dt = - Vm * cos(theta) * sin(phi_V);
    dm_dt = - m_rate;

    dydt = [dV_dt;
        dtheta_dt;
        dphi_V_dt;
        dxm_dt;
        dym_dt;
        dzm_dt;
        dm_dt];
end