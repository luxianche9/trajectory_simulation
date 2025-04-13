function mza = ATM_mza(ma, alpha, xg, L)
    % 阻力系数
    persistent ma_interp alpha_interp mza_interp xg_interp xg0
    if isempty(ma_interp)
        ma_interp = 0.1:0.1:0.9;
        alpha_interp = deg2rad(0:2:10);
        mza_interp = readmatrix('data/ATM/ATM_mza.xlsx');
        xg_interp = readmatrix('data/ATM/ATM_xg.xlsx');
        xg0 = xg_interp(2, 1);
    end

    alpha = max(min(alpha, max(alpha_interp)), -max(alpha_interp));
    ma = max(min(ma, max(ma_interp)), min(ma_interp));
    
    % 第一个参数对应插值表的某一列, 第二个参数对应插值表的某一行
    if alpha >= 0
        mza = interp2(ma_interp, alpha_interp, mza_interp', ma, alpha, "spline");
    else
        mza = - interp2(ma_interp, alpha_interp, mza_interp', ma, - alpha, "spline");
    end
    % 修正重心变化
    cy = ATM_Cy(ma, alpha);
    mza = mza + cy * (xg - xg0) / L;
end