function Cx = ATM_Cx(ma, alpha)
    % 阻力系数
    persistent ma_interp alpha_interp Cx_interp
    if isempty(ma_interp)
        ma_interp = 0.1:0.1:0.9;
        alpha_interp = deg2rad(0:2:10);
        Cx_interp = readmatrix('data/ATM/ATM_Cx.xlsx');
    end

    alpha = max(min(alpha, max(alpha_interp)), - max(alpha_interp));
    alpha = abs(alpha);
    ma = max(min(ma, max(ma_interp)), min(ma_interp));
    % 第一个参数对应插值表的某一列, 第二个参数对应插值表的某一行
    Cx = interp2(ma_interp, alpha_interp, Cx_interp', ma, alpha, "makima");
end