function Cy = ATM_Cy(ma, alpha)
    persistent ma_interp alpha_interp Cy_interp
    if isempty(ma_interp)
        ma_interp = 0.1:0.1:0.9;
        alpha_interp = deg2rad(0:2:10);
        Cy_interp = readmatrix('data/ATM/ATM_Cy.xlsx');
    end

    alpha = max(min(alpha, max(alpha_interp)), - max(alpha_interp));
    ma = max(min(ma, max(ma_interp)), min(ma_interp));
    % 第一个参数对应插值表的某一列, 第二个参数对应插值表的某一行
    if alpha >= 0
        Cy = interp2(ma_interp, alpha_interp, Cy_interp', ma, alpha, "spline");
    else
        Cy = - interp2(ma_interp, alpha_interp, Cy_interp', ma, - alpha, "spline");
    end
end