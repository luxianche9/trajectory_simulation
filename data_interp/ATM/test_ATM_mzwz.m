% 定义马赫数和攻角范围
ma_interp = 0:0.1:2;
alpha_interp = deg2rad(-30:1:30);

% 创建网格
[ma_grid, alpha_grid] = meshgrid(ma_interp, alpha_interp);

y_grid = arrayfun(@(ma, alpha) ATM_mzwz(ma, alpha, 0.9381), ma_grid, alpha_grid);
% 重心变化修正
y_grid2 = arrayfun(@(ma, alpha) ATM_mzwz(ma, alpha, 0.8896), ma_grid, alpha_grid);

% 绘制三维曲面
figure;
hold on;
surf(ma_grid, alpha_grid, y_grid);
surf(ma_grid, alpha_grid, y_grid2);
xlabel('ma');
ylabel('alpha(rad)');
zlabel('mz^omega_z');
title('阻尼力矩系数的三维曲面图');
colorbar;

ma_interp = 0.1:0.1:0.9;
alpha_interp = deg2rad(0:2:10);
% 标记插值点
y_interp = readmatrix('data/ATM/ATM_mzwz.xlsx');
for i = 1:length(ma_interp)
    for j = 1:length(alpha_interp)
        plot3(ma_interp(i), alpha_interp(j), y_interp(i, j), 'ro', 'MarkerFaceColor', 'r');
    end
end
y_interp = readmatrix('data/ATM/ATM_mzwz2.xlsx');
for i = 1:length(ma_interp)
    for j = 1:length(alpha_interp)
        plot3(ma_interp(i), alpha_interp(j), y_interp(i, j), 'ro', 'MarkerFaceColor', 'r');
    end
end
view(3);
hold off;
