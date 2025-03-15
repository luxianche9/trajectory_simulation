function dydt = Target_Dynamics(~, y, pattern)
    % 目标运动方程组
    % pattern: 目标运动模式 ('circle', 'straight', 'stationary')
    % y = [V, theta, phi_V, x, y, z]
    
    V = y(1);
    theta = y(2);    % 弹道倾角
    phi_V = y(3);    % 弹道偏角

    dydt = zeros(6,1); % 初始化导数向量

    if strcmp(pattern, 'circle')
        % 目标绕圈飞行
        omega = 0.1;  % 角速度 (rad/s)

        dydt(1) = 0;
        dydt(2) = 0;
        dydt(3) = omega;
        dydt(4) = V * cos(theta) * cos(phi_V);
        dydt(5) = V * sin(theta);
        dydt(6) = - V * cos(theta) * sin(phi_V);


    elseif strcmp(pattern, 'straight')
        % 目标直线飞行
        dydt(1) = 0;
        dydt(2) = 0;
        dydt(3) = 0;
        dydt(4) = V * cos(theta) * cos(phi_V);
        dydt(5) = V * sin(theta);
        dydt(6) = - V * cos(theta) * sin(phi_V);

    else
        % 目标静止
        dydt(1) = 0;
        dydt(2) = 0;
        dydt(3) = 0;
        dydt(4) = 0;
        dydt(5) = 0;
        dydt(6) = 0;
    end
end
