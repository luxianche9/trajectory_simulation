function rho = Air_Density(h)
    % 输入参数：
    %   h - 高度（单位：米，可接受标量或数组）
    % 输出参数：
    %   rho - 大气密度（单位：kg/m³）
    
    % 国际标准大气（ISA）参数定义
    R = 287.058;        % 比气体常数（J/(kg·K)）
    g0 = 9.80665;       % 重力加速度（m/s²）
    T0 = 288.15;        % 海平面温度（K）
    P0 = 101325;        % 海平面压力（Pa）
    
    % 定义大气分层参数（高度单位为米）
    layers = [
        0,      11000,   -6.5e-3;    % 对流层（0-11 km）
        11000,  20000,   0.0;        % 平流层下层（11-20 km）
        20000,  32000,   1.0e-3;     % 平流层上层（20-32 km）
        32000,  47000,   2.8e-3;     % 中间层下层（32-47 km）
        47000,  51000,   0.0;        % 中间层上层（47-51 km）
        51000,  71000,   -2.8e-3;    % 热层下层（51-71 km）
        71000,  84852,   -2.0e-3     % 热层上层（71-84.852 km）
    ];
    
    % 初始化温度和压力
    T = T0;
    P = P0;
    
    % 遍历所有分层计算温度和压力
    for i = 1:size(layers, 1)
        h_base = layers(i, 1);       % 当前层基准高度
        h_top = layers(i, 2);        % 当前层顶部高度
        lapse_rate = layers(i, 3);   % 温度递减率（K/m）
        
        % 计算当前层的高度范围
        delta_h = min(h, h_top) - h_base;
        valid = (h > h_base);        % 筛选需要处理的高度
        
        % 计算温度
        T_layer = T + lapse_rate * delta_h;
        T = T_layer .* valid + T .* ~valid;
        
        % 计算压力
        if lapse_rate == 0
            % 等温层
            P = P .* exp(-g0/(R*T) .* delta_h) .* valid + P .* ~valid;
        else
            % 非等温层
            P = P .* (T ./ (T + lapse_rate * delta_h)).^(g0/(lapse_rate*R)) .* valid + P .* ~valid;
        end
        
        % 更新基准温度和压力
        T_base = T;
        P_base = P;
        h = max(h - (h_top - h_base), 0); % 处理剩余高度
    end
    
    % 计算密度（理想气体定律）
    rho = P ./ (R * T);
    
    % 处理超出模型范围的高度（84.852 km以上设为0）
    rho(h > 84852) = 0;
end