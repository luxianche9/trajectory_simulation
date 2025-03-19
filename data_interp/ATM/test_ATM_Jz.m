clear;clc;close all;
% 测试 Thrust 函数并绘制推力曲线
t = linspace(0, 50, 100);

% 计算每个时间点的推力
Jz_values = arrayfun(@ATM_Jz, t);

% 绘制推力曲线
figure;
plot(t, Jz_values);
xlabel('时间(s)');
ylabel('转动惯量(kg*m^2)');
title('导弹转动惯量J_z');
grid on;

hold on;
data = readmatrix('data/ATM/ATM_Jz.xlsx');
t_interp = data(1, :);
data_interp = data(2, :);
for i = 1:length(t_interp)
    plot(t_interp(i), data_interp(i),'ro',DisplayName='插值点');
end
