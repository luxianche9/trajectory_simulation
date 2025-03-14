% 定义微分方程 dy/dx = -2x
f = @(x, y) -2 * x;

% 定义初始条件和步长
x0 = 0;
y0 = 1;
dx = 0.1;
xf = 5;

% 定义终止条件，当 y < 0 时终止
event = @(x, y) y < 0;

% 使用欧拉预测矫正法求解微分方程
[x, y] = ode_EPC(x0, dx, xf, y0, f, event);

% 绘制数值解
figure;

axis equal;
xlabel('x');
ylabel('y');
title('微分方程 dy/dx = -2x 的数值解');
plot(x, y, 'ro-', 'DisplayName', '数值解');
grid on;
legend;

