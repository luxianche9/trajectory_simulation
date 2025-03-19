function a = Sound_Speed(h)
    % ISA 模型计算大气温度和声速
    % 输入: h - 高度 (米)
    % 输出: a - 声速 (m/s)
    
    % 物理常数
    T0 = 288.15; % 海平面温度 (K)
    P0 = 101325; % 海平面压力 (Pa)
    rho0 = 1.225; % 海平面密度 (kg/m^3)
    g = 9.80665; % 重力加速度 (m/s^2)
    R = 287.05; % 空气气体常数 (J/(kg·K))
    gamma = 1.4; % 比热比
    
    % ISA 分层数据 [起始高度, 温度梯度, 基准温度]
    layers = [
        0,     -6.5,  288.15;
        11000,  0,    216.65;
        20000,  1.0,  216.65;
        32000,  2.8,  228.65;
        47000,  0,    270.65;
        51000, -2.8,  270.65;
        71000, -2.0,  214.65
    ];
    
    % 计算温度
    T = T0; % 默认温度
    for i = 1:size(layers, 1)-1
        h_base = layers(i, 1);
        L = layers(i, 2) / 1000; % K/m
        T_base = layers(i, 3);
        h_next = layers(i+1, 1);
        
        if h < h_next
            if L ~= 0
                T = T_base + L * (h - h_base);
            else
                T = T_base;
            end
            break;
        end
    end
    
    % 计算声速
    a = sqrt(gamma * R * T);
end
