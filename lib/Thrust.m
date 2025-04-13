function thrust = Thrust(t)
    % 使用样条插值计算任意时间的推力

    load('data/thrust.mat'); % 加载推力数据
    t_interp = linspace(0, 2, length(thrust)); % 生成插值时间点
    
    if t <= 2
        thrust = interp1(t_interp, thrust, t, 'spline'); % 使用样条插值计算推力
    else
        thrust = 0;
    end
end