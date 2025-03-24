function Cx = SAM_Cx(ma, alpha)
    % 定义阻力因数Cx的插值数据
    alpha_interp = deg2rad([2, 4, 6, 8, 10]);
    ma_interp = [1.5, 2.1, 2.7, 3.3, 4.0];
    Cx_interp = [
        0.0430, 0.0511, 0.0651, 0.0847, 0.1120;  % Ma=1.5
        0.0360, 0.0436, 0.0558, 0.0736, 0.0973;  % Ma=2.1
        0.0308, 0.0372, 0.0481, 0.0641, 0.0849;  % Ma=2.7
        0.0265, 0.0323, 0.0419, 0.0560, 0.0746;  % Ma=3.3
        0.0222, 0.0272, 0.0356, 0.0478, 0.0644]; % Ma=4.0

    alpha = max(min(alpha, max(alpha_interp)), - max(alpha_interp));
    alpha = abs(alpha);
    ma = max(min(ma, max(ma_interp)), min(ma_interp));
    % 第一个参数对应插值表的某一列, 第二个参数对应插值表的某一行
    Cx = interp2(ma_interp, alpha_interp, Cx_interp', ma, alpha, "makima");
end