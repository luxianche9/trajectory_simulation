function mza = ATM_mza(ma, alpha, xg, L)
    % 阻力系数
    ma_interp = 0.1:0.1:0.9;
    alpha_interp = deg2rad(0:2:10);
    mza_interp = readmatrix('data/ATM/ATM_mza.xlsx');
    
    % 第一个参数对应插值表的某一列, 第二个参数对应插值表的某一行
    mza = interp2(ma_interp, alpha_interp, mza_interp', ma, alpha, "spline");

    % 修正重心变化
    xg_interp = readmatrix('data/ATM/ATM_xg.xlsx');
    xg0 = xg_interp(2, 1);
    cy = ATM_Cy(ma, alpha);
    mza = mza + cy * (xg - xg0) / L;
end