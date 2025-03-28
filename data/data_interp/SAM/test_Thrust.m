% 测试 Thrust 函数并绘制推力曲线

% 定义时间范围 (0 到 2 秒)
t = linspace(0, 4, 1000);

% 计算每个时间点的推力
thrust_values = arrayfun(@Thrust, t);

% 绘制推力曲线
figure;
plot(t, thrust_values);
xlabel('时间(s)');
ylabel('推力(N)');
title('发动机推力');
grid on;