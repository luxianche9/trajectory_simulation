% 定义马赫数和攻角范围
Ma_range = linspace(1.5, 4.0, 50);
alpha_range = linspace(1, 10, 50);

% 创建网格
[Ma_grid, alpha_grid] = meshgrid(Ma_range, alpha_range);

% 计算升力线斜率
Cya_grid = arrayfun(@(Ma, alpha) SAM_Cya(Ma, alpha), Ma_grid, alpha_grid);

% 绘制三维曲面
figure;
surf(Ma_grid, alpha_grid, Cya_grid);
xlabel('Ma');
ylabel('Alpha');
zlabel('Cx');
title('阻力因数Cx的三维曲面图');
colorbar;

% 标记插值点
hold on;
alpha_Cya = [1, 2, 4, 6, 8, 10];
ma_Cya = [1.5, 2.0, 2.5, 3.0, 3.5, 4.0];
Cya_table = [
    0.0302, 0.0304, 0.0306, 0.0309, 0.0311, 0.0313;  % Ma=1.5
    0.0279, 0.0280, 0.0284, 0.0286, 0.0288, 0.0290;  % Ma=2.0
    0.0261, 0.0264, 0.0267, 0.0269, 0.0272, 0.0274;  % Ma=2.5
    0.0247, 0.0248, 0.0251, 0.0254, 0.0257, 0.0259;  % Ma=3.0
    0.0226, 0.0227, 0.0231, 0.0233, 0.0236, 0.0238;  % Ma=3.5
    0.0209, 0.0210, 0.0213, 0.0216, 0.0219, 0.0221]; % Ma=4.0

for i = 1:length(ma_Cya)
    for j = 1:length(alpha_Cya)
        plot3(ma_Cya(i), alpha_Cya(j), Cya_table(i, j), 'ro', 'MarkerFaceColor', 'r');
    end
end
hold off;
