function Cy_alpha = SAM_Cya(ma, alpha)
    % 定义升力因数斜率Cy_alpha的插值数据
    alpha_interp = deg2rad([1, 2, 4, 6, 8, 10]);
    ma_interp = [1.5, 2.0, 2.5, 3.0, 3.5, 4.0];
    Cy_interp = [
        0.0302, 0.0304, 0.0306, 0.0309, 0.0311, 0.0313;  % Ma=1.5
        0.0279, 0.0280, 0.0284, 0.0286, 0.0288, 0.0290;  % Ma=2.0
        0.0261, 0.0264, 0.0267, 0.0269, 0.0272, 0.0274;  % Ma=2.5
        0.0247, 0.0248, 0.0251, 0.0254, 0.0257, 0.0259;  % Ma=3.0
        0.0226, 0.0227, 0.0231, 0.0233, 0.0236, 0.0238;  % Ma=3.5
        0.0209, 0.0210, 0.0213, 0.0216, 0.0219, 0.0221]; % Ma=4.0
    
    alpha = max(min(alpha, max(alpha_interp)), - max(alpha_interp));
    ma = max(min(ma, max(ma_interp)), min(ma_interp));
    alpha = abs(alpha);
    % 第一个参数对应插值表的某一列, 第二个参数对应插值表的某一行
    Cy_alpha = interp2(ma_interp, alpha_interp, Cy_interp', ma, alpha, "spline");
end