function [x, y] = ode_EPC(x0, dx, xf, y0, f, event)
    % ode_EPC 使用欧拉预测矫正法求解微分方程
    % 输入参数：
    % x0 - 初始时间
    % dx - 时间步长
    % xf - 最大时间
    % y0 - 初始状态
    % f - 函数句柄，表示微分方程 dy/dx = f(x, y)
    % event - 终止条件函数句柄，格式为 @(x, y) event(x, y)
    % 输出参数：
    % x - 时间数组(行向量)
    % y - 状态数组

    % 默认终止条件为无限制
    if nargin < 6
        event = @(x, y) false;
    end
    
    % 初始化时间和状态
    x = x0:dx:xf;
    y = zeros(length(y0), length(x));
    y(:, 1) = y0;

    for i = 1:length(x) - 1
        % 检查终止条件
        if event(x(i), y(:, i))
            break;
        end
        y_pred = y(:, i) + dx * f(x(i), y(:, i));
        y_corr = y(:, i) + dx * (f(x(i), y(:, i)) + f(x(i + 1), y_pred)) / 2;
        
        % 更新状态
        y(:, i + 1) = y_corr;
    end

    % 截取有效数据
    x = x(1:i);
    y = y(:, 1:i);

end