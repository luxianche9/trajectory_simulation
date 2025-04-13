function thrust = ATM_thrust(t)
    % 阻力系数
    persistent data t_interp1 thrust_interp1 t_interp2 thrust_interp2
    if isempty(data)
        data = readmatrix('data/ATM/ATM_Thrust.xlsx');
        t_interp1 = data(1, 1:4);
        thrust_interp1 = data(2, 1:4);
        t_interp2 = data(1, 5:end);
        thrust_interp2 = data(2, 5:end);
    end
    
    % 第一个参数对应插值表的某一列, 第二个参数对应插值表的某一行
    if t <= max(t_interp1)
        thrust = interp1(t_interp1, thrust_interp1, t, 'pchip'); % 使用样条插值计算推力
    elseif t <= max(t_interp2)
        thrust = interp1(t_interp2, thrust_interp2, t, 'pchip');
    else
        thrust = 0;
    end

    thrust = thrust * 9.8;
end