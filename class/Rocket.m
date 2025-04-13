classdef Rocket < handle
    properties
        P_mag
        I_sp
        t_burn

        omegaE

        recoder
        t0
        dt
        tf
    end
    methods
        function obj = Rocket(t0, dt, tf)
            obj.P_mag = 20000; % 发动机推力 (N)
            obj.I_sp = 2842; % 发动机比冲 (s)
            obj.t_burn = 30; % 发动机工作时间 (s)
            obj.omegaE = 7.2921159e-5; % 地球自转角速度 (rad/s)
            % obj.omegaE = 0;
            obj.t0 = t0;
            obj.dt = dt;
            obj.tf = tf;
            recoder_length = round((tf- t0)/dt) + 10;
            obj.recoder.P = zeros(3, recoder_length); % 推力矢量
            obj.recoder.r_rel = zeros(1, recoder_length); % 相对位置
            obj.recoder.r_T = zeros(3, recoder_length); % 目标位置矢量
            obj.recoder.V_R = zeros(3, recoder_length); % 导弹速度矢量
        end

        function value = HitGround(~, ~, y)
            R_Earth = 6371.4; % 地球半径 (km)
            r = y(1:3);
            if norm(r) < R_Earth
                value = 1; % 导弹击中地面
                disp("导弹击中地面！");
            else
                value = 0; % 导弹未击中地面
            end
        end

        function dydt = RocketDynamics(obj, t, y, r_T0, gama_T)
            disp(t);
            r_M = y(1:3);
            v_M = y(4:6);
            m = y(7);


            % 目标位置
            omega_E = obj.omegaE; % 地球自转角速度 (rad/s)
            R = [cos(omega_E * t) -sin(omega_E * t) 0;
                sin(omega_E * t)  cos(omega_E * t) 0;
                0                 0                1];
            r_T = R * r_T0; % 目标位置矢量


            % 重力加速度
            mu = 398600; % 地球引力常数 (km^3/s^2)
            g = - mu / norm(r_M)^3 * r_M;

            % 发动机推力
            V_R = obj.RocketGuidance(r_M, r_T, gama_T);
            V_g = V_R - v_M;  % 相对速度矢量
            if norm(V_g) < 1e-6
                P_dir = zeros(3,1); % 或者保持上一次方向
            else
                P_dir = V_g / norm(V_g);
            end
            % 确保推力只在燃烧阶段施加
            if t <= obj.t_burn
                P = obj.P_mag * P_dir;
            else
                P = 0 * P_dir;
            end
            
            % 微分方程组
            a = g + P / m;
            m_c = - 1 / (obj.I_sp * 9.81) * norm(P); % 燃料消耗率 (kg/s)
            dydt = [v_M; a; m_c];

            i = round((t - obj.t0) / obj.dt) + 1;
            obj.recoder.r_T(:, i) = r_T; % 记录目标位置矢量
            obj.recoder.r_rel(i) = norm(r_M - r_T) * 1000; % 记录相对位置
            obj.recoder.P(:, i) = P; % 记录推力方向
            obj.recoder.V_R(:, i) = V_R; % 记录导弹速度矢量
        end

        function V_R = RocketGuidance(obj, r_M, r_T, gama_T)
            % 获得导弹所需速度大小与俯仰角（航迹角）
            [V_R_mag, gama_R] = obj.func2(r_M, r_T, gama_T);  % 用户自定义

            % 获取导弹位置的地理坐标（经纬度）
            [lambda_M, phi_M] = obj.func4(r_M);

            % 计算目标与导弹之间的方位角 alpha
            alpha = obj.func3(r_T, r_M);

            fprintf("解算:\nV_R: %.2f km/s gama_R: %.2f deg\nlambda_M: %.2f deg phi_M: %.2f deg\n", ...
                   V_R_mag, rad2deg(gama_R), rad2deg(lambda_M), rad2deg(phi_M));
            fprintf("alpha: %.2f deg\n", rad2deg(alpha));

            % 计算当地北东坐标系下的速度分量
            V_local = [V_R_mag * cos(gama_R) * cos(alpha);
                       V_R_mag * sin(gama_R);
                       V_R_mag * cos(gama_R) * sin(alpha)];
        
            % 构造从当地北东坐标系到地心惯性坐标系的转换矩阵
            L = eul2rotm([phi_M, -lambda_M, 0],'ZXY')';
            % 转换为地心坐标系下的速度矢量
            V_R_temp =L * V_local;
            V_R = [V_R_temp(2); V_R_temp(3); V_R_temp(1)];
        end

        function [lambda, phi] = func4(~, r)
            % 输入: 位置矢量
            % 输出: 经纬度（球面坐标系）
            x = r(1); y = r(2); z = r(3);
            lambda = atan(y/x);           % 经度
            phi = asin(z / norm(r));        % 纬度（球面模型）
        end
        
        function alpha = func3(obj, r_T, r_M)
            % 输入: 目标和导弹位置矢量
            % 输出: 目标和导弹之间的方位角 alpha
            % 使用球面坐标系计算
            beta = acos(dot(r_M, r_T) / (norm(r_M) * norm(r_T)));  % 球面中心夹角
        
            [lambda_M, phi_M] = obj.func4(r_M);
            [lambda_T, phi_T] = obj.func4(r_T);
        
            d_lambda = lambda_T - lambda_M;
            sin_alpha = sin(abs(d_lambda)) * cos(phi_T) / sin(beta);
            cos_alpha = (sin(phi_T) - cos(beta) * sin(phi_M)) / (cos(phi_M) * sin(beta));
        
            % 根据象限判断 alpha
            if abs(sin_alpha) <= abs(cos_alpha)
                if cos_alpha >= 0
                    alpha = asin(sin_alpha);
                else
                    alpha = pi * sign(sin_alpha) - asin(sin_alpha);
                end
            else
                alpha = sign(sin_alpha) * acos(cos_alpha);
            end
        end
        

        function [V_R_mag, gama_R] = func2(obj, r_M, r_T, gama_T)
            % 输入: 导弹和目标的失径, 在目标点的速度倾角
            % 输出: 考虑地球自转, 导弹点的速度大小和速度倾角
            % 使用迭代求解
            mu = 398600; % 地球引力常数 (km^3/s^2)

            [~, p, ~, t, ~] = obj.func1(r_M, r_T, gama_T, 0);
            while true
                [~, p_new, ~, t_new, ~] = obj.func1(r_M, r_T, gama_T, t);
                if abs(p_new - p) <= 0.00001
                    break;
                end
                fprintf("t: %.2f, p: %.2f\n", t_new, p_new);
                t = t_new;
                p = p_new;
            end
            t = t_new;
            [~, p, theta_T, ~, beta] = obj.func1(r_M, r_T, gama_T, t);
            
            % V_R_mag = sqrt(mu / p * (1 + e^2 + 2 * e * cos(theta_T)));
            gama_R = atan((p - norm(r_M)) / p * tan(theta_T - beta));
            V_R_mag = sqrt(mu * p) / (norm(r_M) * cos(gama_R));
        end

        function [e, p, theta_T, t, beta] = func1(obj, r_M, r_T, gama_T, t)
            % 输入: 导弹和目标位置失径, 在目标点的速度倾角
            % 输出: 过这两点的轨道的轨道参数, 导弹到目标的运动时间
            mu = 398600; % 地球引力常数 (km^3/s^2)

            omega_E = obj.omegaE; % 地球自转角速度 (rad/s)
            R = [cos(omega_E * t) -sin(omega_E * t) 0;
                 sin(omega_E * t)  cos(omega_E * t) 0;
                 0                 0                1];
            r_T = R * r_T; % 目标位置矢量
            beta = acos((r_M' * r_T) / (norm(r_M) * norm(r_T)));
            p = norm(r_M) * (1 - cos(beta)) / ...
                (1 - norm(r_M) / norm(r_T) *  ...
                (cos(beta) + sin(beta) * tan(gama_T)));
            theta_T = atan(p * tan(gama_T)/(p - norm(r_T)));
            e = (p / norm(r_T) - 1) / cos(theta_T);
            theta_M = theta_T - beta;
            E_M = 2 * atan(sqrt((1 - e) / (1 + e)) * tan(theta_M / 2));
            E_T = 2 * atan(sqrt((1 - e) / (1 + e)) * tan(theta_T / 2));
            M_T = E_T - e * sin(E_T);
            M_M = E_M - e * sin(E_M);
            a = p / (1 - e^2);
            t = sqrt(a^3 / mu) * (M_T - M_M);
        end
    end
end