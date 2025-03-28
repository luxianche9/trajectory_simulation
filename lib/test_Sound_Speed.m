% 测试 Sound_Speed 函数并绘制 a 与 h 的关系图

% 定义高度范围 (0 到 80,000 米)
h = linspace(0, 80000, 1000);

% 计算每个高度的声速
a = arrayfun(@Sound_Speed, h);

% 绘制 a 与 h 的关系图
figure;
plot(h./1000, a, 'r-', LineWidth=2);
xlabel('高度 (km)');
ylabel('声速 (m/s)');
title('声速模型');
grid on;