function y = ATM_mzdz(ma, dz)
    persistent ma_interp dz_interp y_interp
    if isempty(ma_interp)
        ma_interp = 0.1:0.1:0.9;
        dz_interp = deg2rad(0:10:30);
        y_interp = readmatrix('data/ATM/ATM_mzdz.xlsx');
    end
    dz = max(min(dz, max(dz_interp)), -max(dz_interp));
    ma = max(min(ma, max(ma_interp)), min(ma_interp));
    % 第一个参数对应插值表的某一列, 第二个参数对应插值表的某一行
    if dz >= 0
        y = interp2(ma_interp, dz_interp, y_interp', ma, dz, "spline");
    elseif dz < 0
        y = - interp2(ma_interp, dz_interp, y_interp', ma, - dz, "spline");
    end
end