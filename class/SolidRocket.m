classdef SolidRocket
    properties
        T
        I_sp
        t_burn
    end
    methods
        function obj = SolidRocket()
            obj.T = 20000; % 发动机推力 (N)
            obj.I_sp = 2842; % 发动机比冲 (s)
            obj.t_burn = 30; % 发动机工作时间 (s)
        end

        function dydt = RocketDynamics(obj, t, y, T)
            r = y(1:3);
            v = y(4:6);
            m = y(7);

            % 重力加速度
            g = -mu / r_norm^3 * r;

            thrust_dir = Rocket_Guidance(y, r_p, theta_p);
            
            % 确保推力只在燃烧阶段施加
            if t <= obj.t_burn
                thrust_mag = obj.T;
                m_c = -T / (ojb.I_sp * 9.81);
            else
                thrust_mag = 0;
                m_c = 0; % 质量不再减少
            end
            
            thrust = thrust_mag * thrust_dir;
            % 总加速度
            a = g + thrust / m;
            dydt = [v; a; m_c];
        end

        function u = RocketGuidance(t, y)
            mu = 398600; % 地球引力常数 (km^3/s^2)
            r = y(1:3);
            v = y(4:6);
            r_norm = norm(r);
            r_p_norm = norm(r_p);
            
            % 计算射程角β_e
            cos_beta_e = dot(r, r_p) / (r_norm * r_p_norm);
            beta_e = acos(cos_beta_e);
            
            % 计算半通径p
            denominator = 1 - (r_norm/r_p_norm) * (cos_beta_e + tan(theta_p)*sin(beta_e));
            p = (r_norm * (1 - cos_beta_e)) / denominator;
            
            % 计算β_p和θ_l0
            beta_p = atan(tan(theta_p) / (1 - r_p_norm/p));
            theta_l0 = atan((1 - r_norm/p) * tan(beta_p - beta_e));
            
            % 需要速度大小
            v_r = sqrt(mu * p) / (r_norm * cos(theta_l0));
            
            % 计算经纬度
            lambda0 = atan2(r(2), r(1));
            lambda_p = atan2(r_p(2), r_p(1));
            phi0 = asin(r(3)/r_norm);
            phi_p = asin(r_p(3)/r_p_norm);
            
            % 计算方位角α_e
            delta_lambda = lambda_p - lambda0;
            sin_alpha_e = sin(abs(delta_lambda)) * cos(phi_p) / sin(beta_e);
            cos_alpha_e = (sin(phi_p) - cos_beta_e*sin(phi0)) / (cos(phi0)*sin(beta_e));
            
            % 确定α_e象限
            if abs(sin_alpha_e) <= abs(cos_alpha_e)
                if cos_alpha_e >= 0
                    alpha_e = asin(sin_alpha_e);
                else
                    alpha_e = pi*sign(sin_alpha_e) - asin(sin_alpha_e);
                end
            else
                alpha_e = sign(sin_alpha_e)*acos(cos_alpha_e);
            end
            
            % 分解到当地坐标系
            v_rx1 = v_r * sin(theta_l0);
            v_ry1 = v_r * cos(theta_l0) * sin(alpha_e);
            v_rz1 = v_r * cos(theta_l0) * cos(alpha_e);
            
            % 转换到惯性系
            C_z = [cos(-lambda0), -sin(-lambda0), 0;
                   sin(-lambda0),  cos(-lambda0), 0;
                   0, 0, 1];
            C_y = [cos(phi0), 0, -sin(phi0);
                   0, 1, 0;
                   sin(phi0), 0, cos(phi0)];
            v_r_inertial = C_z * C_y * [v_rx1; v_ry1; v_rz1];
            
            % 计算推力方向
            v_g = v_r_inertial - v;
            v_g_norm = norm(v_g);
            u = v_g / v_g_norm; % 单位矢量
        end
    end
end