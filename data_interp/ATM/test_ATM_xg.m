clear;clc;close all;
% 测试 Thrust 函数并绘制推力曲线
t = linspace(0, 50, 1000);

% 计算每个时间点的推力
values = arrayfun(@ATM_xg, t);

% 绘制推力曲线
figure;
plot(t, values);
xlabel('时间(s)');
ylabel('质心位置(m)');
title('质心位置(起自头部)');
grid on;

hold on;
data = readmatrix('data/ATM/ATM_xg.xlsx');
t_interp = data(1, :);
y_interp = data(2, :);
for i = 1:length(t_interp)
    plot(t_interp(i), y_interp(i),'ro',DisplayName='插值点');
end