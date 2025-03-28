classdef Target
    properties
        pattern % 目标运动模式 ('circle', 'straight', 'stationary', 'random')
    end

    methods
        function obj = Target(pattern)
            % 构造函数
            obj.pattern = pattern; % 目标运动模式
        end

        function dstates_dt = Target_Dynamics(obj, states)
            % y = [V, theta, phi_V, x, y, z]
            V = states(1);
            theta = states(2);    % 弹道倾角
            phi_V = states(3);    % 弹道偏角
        
            dstates_dt = zeros(6, 1); % 初始化导数向量
            if strcmp(obj.pattern, 'circle')
                % 目标绕圈飞行
                omega = deg2rad(10);  % 角速度 (rad/s)
                dstates_dt(1) = 0;
                dstates_dt(2) = 0;
                dstates_dt(3) = omega;
            elseif strcmp(obj.pattern, 'straight')
                % 目标直线飞行
                dstates_dt(1) = 0;
                dstates_dt(2) = 0;
                dstates_dt(3) = 0;
            elseif strcmp(obj.pattern, 'random')
                % 目标随机飞行
                omega = deg2rad(240);
                omega1 = 2 * (rand-0.5) * omega;
                omega2 = 2 * (rand-0.5) * omega;
                dstates_dt(1) = 0;
                dstates_dt(2) = omega1;
                dstates_dt(3) = omega2;
            else
                % 目标静止
                dstates_dt(1) = 0;
                dstates_dt(2) = 0;
                dstates_dt(3) = 0;
            end

            dstates_dt(4) = V * cos(theta) * cos(phi_V);
            dstates_dt(5) = V * sin(theta);
            dstates_dt(6) = - V * cos(theta) * sin(phi_V);
        end    
    end
end