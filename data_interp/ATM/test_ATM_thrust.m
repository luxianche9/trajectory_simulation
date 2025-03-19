clear;clc;close all;
% 测试 Thrust 函数并绘制推力曲线
t = linspace(0, 50, 1000);

% 计算每个时间点的推力
thrust_values = arrayfun(@ATM_thrust, t);

% 绘制推力曲线
figure;
plot(t, thrust_values);
xlabel('时间(s)');
ylabel('推力(N)');
title('发动机推力');
grid on;

hold on;
data = readmatrix('data/ATM/ATM_Thrust.xlsx');
t_interp = data(1, :);
thrust_interp = data(2, :);
thrust_interp = thrust_interp * 9.8;
for i = 1:length(t_interp)
    plot(t_interp(i), thrust_interp(i),'ro',DisplayName='插值点');
end