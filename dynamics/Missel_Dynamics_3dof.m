function dydt = Missel_Dynamics_3dof(t, y, delta_z)
    % 导弹动力学方程组 纵向平面
    % y = [Vm, theta, wz, xm, ym, nu, m]
    Vm = y(1);
    theta = y(2);
    wz = y(3);
    xm = y(4);
    ym = y(5);
    nu = y(6);
    m = y(7);
    alpha = nu - theta;
    % fprintf('Vm:%.2f theta:%.2f x:%.2f y:%.2f nu:%.2f\n',Vm , theta, xm, ym, nu);
    
    % 导弹
    Jz = ATM_Jz(t);
    xg = ATM_xg(t);
    S_ref = 0.0227; % 特征面积(m^2)
    L_ref = 1.8; % 特征长度(m)
    L_wing = 0.5; % 翼展(m)
    % 环境
    sound_speed = Sound_Speed(ym);
    ma = Vm / sound_speed;
    rho = Air_Density(ym);
    q = 1/2 * rho * Vm ^ 2;
    % 重力
    g = 9.8;
    G = m * g;
    % 推力
    m_c = ATM_mc(t);
    P = ATM_thrust(t);
    % 气动力
    Cx = ATM_Cx(ma, alpha);
    Cy = ATM_Cy(ma, alpha);
    mza = ATM_mza(ma, alpha, xg, L_ref);
    mzdz = ATM_mzdz(ma, delta_z);
    mzwz = ATM_mzwz(ma, alpha, xg); % 归一化omega * L /V
    Y = q * S_ref * Cy;
	X = q * S_ref * Cx;
    Mz = (mza + mzdz + mzwz*wz*L_wing/Vm) * q * S_ref * L_wing;

    % 导弹动力学方程组
    dV_dt = (P * cos(alpha) - X - G * sin(theta)) / m;
    dtheta_dt = (P * sin(alpha) + Y - G * cos(theta)) ...
                / (m * Vm);
    dwz_dt = Mz / Jz;
    dxm_dt = Vm * cos(theta);
    dym_dt = Vm * sin(theta);
    dnu_dt = wz;
    dm_dt = - m_c;

    dydt = [dV_dt;
        dtheta_dt;
        dwz_dt;
        dxm_dt;
        dym_dt;
        dnu_dt;
        dm_dt];
    
    disp(t);

    % fprintf(['时间%.4f\n'...
    %         '速度%.2f 马赫数%.2f 动压%.2f\n'...
    %         '射程%.2f 高度%.2f 弹道倾角%.2f(deg) 俯仰角%.2f(deg)\n'...
    %         '俯仰力矩%.2f 转动角速度%.2f\n'...
    %         '质量%.2f 质心位置%.2f 质量流量%.2f\n'...
    %         '攻角%.2f(deg) 升力系数%.2f 升力%.2f 阻力系数Cx%.2f 阻力%.2f 推力%.2f\n\n'], ...
    %         t, ...
    %         Vm, ma, q, ...
    %         xm, ym, rad2deg(theta), rad2deg(nu), ...
    %         Mz, wz, ...
    %         m, xg, m_c, ...
    %         rad2deg(alpha), Cy, Y, Cx, X, P);
end