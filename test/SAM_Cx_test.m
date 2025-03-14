% 定义马赫数和攻角范围
Ma_range = linspace(1.5, 4.0, 50);
alpha_range = linspace(2, 10, 50);

% 创建网格
[Ma_grid, alpha_grid] = meshgrid(Ma_range, alpha_range);

% 计算阻力因数Cx
Cx_grid = arrayfun(@(Ma, alpha) SAM_Cx(Ma, alpha), Ma_grid, alpha_grid);

% 绘制三维曲面
figure;
surf(Ma_grid, alpha_grid, Cx_grid);
xlabel('Ma');
ylabel('Alpha');
zlabel('Cy_alpha');
title('阻力因数Cy_alpha的三维曲面图');
colorbar;

% 标记插值点
hold on;
alpha_Cx = [2, 4, 6, 8, 10];
ma_Cx = [1.5, 2.1, 2.7, 3.3, 4.0];
Cx_table = [
    0.0430, 0.0511, 0.0651, 0.0847, 0.1120;  % Ma=1.5
    0.0360, 0.0436, 0.0558, 0.0736, 0.0973;  % Ma=2.1
    0.0308, 0.0372, 0.0481, 0.0641, 0.0849;  % Ma=2.7
    0.0265, 0.0323, 0.0419, 0.0560, 0.0746;  % Ma=3.3
    0.0222, 0.0272, 0.0356, 0.0478, 0.0644]; % Ma=4.0

for i = 1:length(ma_Cx)
    for j = 1:length(alpha_Cx)
        plot3(ma_Cx(i), alpha_Cx(j), Cx_table(i, j), 'ro', 'MarkerFaceColor', 'r');
    end
end
hold off;
