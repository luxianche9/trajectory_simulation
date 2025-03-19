function y = ATM_xg(t)
    % 阻力系数
    data = readmatrix('data/ATM/ATM_xg.xlsx');
    t_interp = data(1, :);
    y_interp = data(2, :);
    if t <= max(t_interp)
        y = interp1(t_interp, y_interp, t, 'pchip');
    else
        y = min(y_interp);
    end
end