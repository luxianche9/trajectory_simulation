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

    Cy_alpha = 10.4;
    Cy_dz = 1.51;
    Cz_beta = 10.4;
    Cz_dy = 1.51;

    alpha = (m*Vm*dtheta_dt_target + G * cos(theta)) / (P + 1/2 * rho *Vm^2 * Cy_alpha * S_ref);
    beta = (m*Vm*dphi_V_dt_target*cos(theta)) / (P + 1/2 * rho *Vm^2 * Cz_beta * S_ref);

    Cy = Cy_alpha * alpha + Cy_dz * delta_z;
    Cx = 0.437 + 7.01*alpha*delta_z + 17.3*alpha^2 + 2.41*delta_z^2;
    Cz = Cz_beta * beta + Cz_dy * delta_y;
    
    Y = Cy * 1/2 * rho * Vm^2 * S_ref;
    X = Cx * 1/2 * rho * Vm^2 * S_ref;
    Z = Cz * 1/2 * rho * Vm^2 * S_ref;

    % 导弹动力学方程组
    dV_dt = (P*cos(alpha)*cos(beta) - X - G*sin(theta)) / m;
    dtheta_dt = (P*sin(alpha) + Y - G*cos(theta)) / (m*Vm);
    dphi_V_dt = (P*cos(alpha)*sin(beta) - Z) / (m * Vm * cos(theta));
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