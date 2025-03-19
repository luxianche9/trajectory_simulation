function Cx = ATM_Cx(ma, alpha)
    % 阻力系数
    ma_interp = 0.1:0.1:0.9;
    alpha_interp = deg2rad(0:2:10);
    Cx_interp = readmatrix('data/ATM/ATM_Cx.xlsx');

    % 第一个参数对应插值表的某一列, 第二个参数对应插值表的某一行
    Cx = interp2(ma_interp, alpha_interp, Cx_interp', ma, alpha, "spline");
end