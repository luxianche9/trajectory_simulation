function mc = ATM_mc(t)
    % 阻力系数
    persistent data t_interp1 mc_interp1 t_interp2 mc_interp2
    if isempty(data)
        data = readmatrix('data/ATM/ATM_mc.xlsx');
        t_interp1 = data(1, 1:2);
        mc_interp1 = data(2, 1:2);
        t_interp2 = data(1, 3:end);
        mc_interp2 = data(2, 3:end);
    end
    % 第一个参数对应插值表的某一列, 第二个参数对应插值表的某一行
    if t <= max(t_interp1)
        mc = interp1(t_interp1, mc_interp1, t, 'pchip'); % 使用样条插值计算推力
    elseif t <= max(t_interp2)
        mc = interp1(t_interp2, mc_interp2, t, 'pchip');
    else
        mc = 0;
    end
end