function Jz = ATM_Jz(t)
    % 阻力系数
    data = readmatrix('data/ATM/ATM_Jz.xlsx');
    t_interp = data(1, :);
    Jz_interp = data(2, :);
    if t < max(t_interp)
        Jz = interp1(t_interp, Jz_interp, t, 'spline');
    else
        Jz = min(Jz_interp);
    end
end