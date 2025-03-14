function m_rate = M_rate(t)
    % 使用样条插值计算任意时间的推力

    load('data/m_rate.mat'); % 加载推力数据
    t_interp = linspace(0, 2, length(m_rate)); % 生成插值时间点
    m_rate = interp1(t_interp, m_rate, t, 'spline'); % 使用样条插值计算推力
end