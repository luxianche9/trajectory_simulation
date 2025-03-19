function mzwz = ATM_mzwz(ma, alpha, xg)
    % 阻力系数
    ma_interp = 0.1:0.1:0.9;
    alpha_interp = deg2rad(0:2:10);
    mzwz_interp = readmatrix('data/ATM/ATM_mzwz.xlsx');
    mzwz_interp2 = readmatrix('data/ATM/ATM_mzwz2.xlsx');
    
    % 第一个参数对应插值表的某一列, 第二个参数对应插值表的某一行
    mzwz0 = interp2(ma_interp, alpha_interp, mzwz_interp', ma, alpha, "spline");
    mzwzf = interp2(ma_interp, alpha_interp, mzwz_interp2', ma, alpha, "spline");
    
    xg_interp = readmatrix('data/ATM/ATM_xg.xlsx');
    xg0 = xg_interp(2, 1);
    xgf = xg_interp(2,end);
    
    mzwz = (xg - xg0)/(xgf - xg0) * (mzwzf - mzwz0) + mzwz0;
end