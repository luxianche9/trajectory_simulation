clear;clc;close all;

% 定义马赫数和攻角范围
ma_interp = 0.1:0.1:2;
dz_interp = deg2rad(-30:1:30);

% 创建网格
[ma_grid, dz_grid] = meshgrid(ma_interp, dz_interp);

% 计算阻力因数Cx
y_grid = arrayfun(@(ma, dz) ATM_mzdz(ma, dz), ma_grid, dz_grid);

% 绘制三维曲面
figure;
hold on;
surf(ma_grid, dz_grid, y_grid);
xlabel('ma');
ylabel('dz(rad)');
zlabel('mz^delta_z');
title('俯仰操纵力矩系数的三维曲面图');
colorbar;

% 标记插值点
ma_interp = 0.1:0.1:0.9;
dz_interp = deg2rad(0:10:30);
y_interp = readmatrix('data/ATM/ATM_mzdz.xlsx');
for i = 1:length(ma_interp)
    for j = 1:length(dz_interp)
        plot3(ma_interp(i), dz_interp(j), y_interp(i, j), 'ro', 'MarkerFaceColor', 'r');
    end
end
view(3);
hold off;
