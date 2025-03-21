clear;clc;close all;
% 测试 Thrust 函数并绘制推力曲线
t = 0:0.1:50;

% 计算每个时间点的推力
mc_values = arrayfun(@ATM_mc, t);

% 绘制推力曲线
figure;
plot(t, mc_values);
xlabel('时间(s)');
ylabel('质量秒流率(kg/s)');
title('发动机质量秒流量');
grid on;

hold on;
data = readmatrix('data/ATM/ATM_mc.xlsx');
t_interp = data(1, :);
mc_interp = data(2, :);
for i = 1:length(t_interp)
    plot(t_interp(i), mc_interp(i),'ro',DisplayName='插值点');
end