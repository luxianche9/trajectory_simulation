% 定义马赫数和攻角范围
ma_interp = 0:0.1:2;
alpha_interp = deg2rad(-30:2:30);

% 创建网格
[ma_grid, alpha_grid] = meshgrid(ma_interp, alpha_interp);

% 计算阻力因数Cx
Cx_grid = arrayfun(@(ma, alpha) ATM_Cx(ma, alpha), ma_grid, alpha_grid);

% 绘制三维曲面
figure;
hold on;
surf(ma_grid, alpha_grid, Cx_grid);
xlabel('ma');
ylabel('alpha(rad)');
zlabel('Cx');
title('阻力因数的三维曲面图');
colorbar;

% 标记插值点
% alpha_interp = deg2rad(0:2:10);
% Cx_interp = readmatrix('data/ATM/ATM_Cx.xlsx');
% for i = 1:length(ma_interp)
%     for j = 1:length(alpha_interp)
%         plot3(ma_interp(i), alpha_interp(j), Cx_interp(i, j), 'ro', 'MarkerFaceColor', 'r');
%     end
% end
view(3);
hold off;
