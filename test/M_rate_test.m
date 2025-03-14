% 测试 Thrust 函数并绘制推力曲线

% 定义时间范围 (0 到 2 秒)
t = linspace(0, 4, 1000);

% 计算每个时间点的推力
thrust_values = arrayfun(@M_rate, t);

% 绘制推力曲线
figure;
plot(t, thrust_values);
xlabel('时间 (秒)');
ylabel('质量流量');
title('质量流量与时间的关系');
grid on;