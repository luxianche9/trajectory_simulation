clear; clc; close all
% 定义高度范围（单位：米）
h_range = linspace(0, 85000, 1000);

% 计算对应的空气密度
for i = 1:length(h_range)
    rho_range(i) = Air_Density(h_range(i));
end

% 绘制高度与密度关系图
figure;grid on;hold on;
plot(h_range./1000, rho_range, 'b-', LineWidth=2);
xlabel('高度 (km)');
ylabel('空气密度 (kg/m³)');
title('大气密度模型');

% 标注大气层名称

text(5, 1.2, '对流层', 'HorizontalAlignment', 'center');
text(15, 0.3, '平流层', 'HorizontalAlignment', 'center');
text(30, 0.15, '中间层', 'HorizontalAlignment', 'center');
text(60, 0.07, '热层', 'HorizontalAlignment', 'center');
hold off;
