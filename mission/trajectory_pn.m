function [t, t_x, t_y, m_x, m_y, a_m, a_c, r] = ...
    trajectory_pn(guidance_mode, N, target_maneuver, order)
    % trajectory_pn: 真比例导引法仿真
    % 输入参数: 
    % guidance_mode: 导引律: 'TPN' 或 'PPN'
    % N: 比例导引法参数
    % target_maneuver: 目标机动: 'circle' 或 'straight'
    % order: 自动驾驶仪阶数
    % 输出参数:
    % t: 时间序列
    % t_x: 目标X位置序列
    % t_y: 目标Y位置序列
    % m_x: 导弹X位置序列
    % m_y: 导弹Y位置序列
    % a_m: 导弹加速度序列
    % a_c: 控制加速度序列
    % r: 脱靶量序列

    %% 仿真参数设置
    % 数值积分参数
    dt = 0.0001;% 时间步长 (s)
    t_max = 4.3;% 最大仿真时间 (s)
    step = floor(t_max/dt);% 仿真步数

    % 制导参数
    omega = deg2rad(5);       % 目标转弯速率 (rad/s)
    intercept_threshold = 1;  % 拦截阈值 (m)

    % 自动驾驶仪
    zeta = 0.5;% 阻尼系数
    T = 0.1;% 时间常数

    %% 初始化
    % 导弹
    m_v = 3000;
    m_sig = deg2rad(45);
    m_x = 0;
    m_y = 0;

    % 相对位置
    q = deg2rad(30);
    r0 = 10000;

    % 目标
    t_v = 1000;
    t_sig = deg2rad(90);
    t_x = r0*cos(q);
    t_y = r0*sin(q);

    % 记录数据
    t_list = zeros(1, step);
    m_x_list = zeros(1,step);
    m_y_list = zeros(1,step);
    t_x_list = zeros(1,step);
    t_y_list = zeros(1,step);
    r_list = zeros(1,step);

    a_m_list = zeros(1,step);%a_m
    da_m_list = zeros(1,step);%a_m'

    a_c_list = zeros(1,step);%u

    %% 仿真

    intercepted = false;
    for n = 1:step
        % 记录数据
        t_list(n) = n*dt;
        m_x_list(n) = m_x;
        m_y_list(n) = m_y;
        t_x_list(n) = t_x;
        t_y_list(n) = t_y;

        rx = t_x - m_x;
        ry = t_y - m_y;
        r = norm([rx, ry]);
        r_list(n) = r;

        if r < intercept_threshold
            intercepted = true;
            fprintf('拦截成功于 %.2fs\n',t_list(n));
        end

        % 目标机动
        if strcmp(target_maneuver, 'circle')
            t_sig = t_sig + omega*dt;
            t_sig = mod(t_sig, 2*pi);
        end
        
        % 视线角变化率
        q = atan2(ry, rx);
        drdt = t_v*cos(t_sig - q) - m_v*cos(m_sig - q);
        dqdt = (t_v*sin(t_sig - q) - m_v*sin(m_sig - q))/r;
        
        if strcmp(guidance_mode, 'TPN')
            % 真比例导引法，加速度垂直于视线，大小为N*Vc*dqdt
            a_c_mag = N * (-drdt) * dqdt;
            a_c_dir = [-sin(q); cos(q)];
            a_c = a_c_mag * a_c_dir;
        else
            % 纯比例导引法，加速度垂直于导弹速度，大小为N*Vm*dqdt
            a_c_mag = N * m_v * dqdt;
            a_c_dir = [-sin(m_sig); cos(m_sig)];
            a_c = a_c_mag * a_c_dir;
        end
        a_c_list(n) = a_c_mag;
        
        % 自动驾驶仪动态
        if order == 2
            dda_m = a_c_mag/T^2 - a_m_list(n)/T^2 - 2 * zeta / T * da_m_list(n);
            da_m = da_m_list(n);
    
            a_m_list(n+1) = a_m_list(n) + dda_m * dt;
            da_m_list(n+1) = da_m + a_m_list(n+1) * dt;
    
            a_m = a_m_list(n)*a_c_dir;
        else
            da_m = (a_c_mag - a_m(n))/T;
            
            a_m_list(n+1) = a_m(n) + da_m * dt;

            a_m = a_m_list(n)*a_c_dir;
        end

        % 更新导弹航向角
        v_dir = [cos(m_sig); sin(m_sig)];
        lateral_dir = [-v_dir(2); v_dir(1)];
        a_lateral = dot(a_m, lateral_dir);
        m_sig = m_sig + (a_lateral / m_v) * dt;
        
        % 更新位置
        m_x = m_x + m_v*cos(m_sig)*dt;
        m_y = m_y + m_v*sin(m_sig)*dt;
        t_x = t_x + t_v*cos(t_sig)*dt;
        t_y = t_y + t_v*sin(t_sig)*dt;
    end

    % 截断未使用的数据
    if intercepted
        m_x_list = m_x_list(1:n);
        m_y_list = m_y_list(1:n);
        t_x_list = t_x_list(1:n);
        t_y_list = t_y_list(1:n);
        t_list = t_list(1:n);
        a_m_list = a_m_list(1:n);
        a_c_list = a_c_list(1:n);
        r_list = r_list(1:n);
    end

    t_x = t_x_list;
    t_y = t_y_list;
    m_x = m_x_list;
    m_y = m_y_list;
    a_m = a_m_list;
    a_c = a_c_list;
    t = t_list;
    r = r_list;
end
