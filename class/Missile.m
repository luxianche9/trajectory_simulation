classdef Missile < handle % 继承Handle, 实现引用传递
    properties
        % 导弹参数
        S_ref % 特征面积(m^2)
        L_ref % 特征长度(m)
        L_wing % 翼展(m)
        R_destroy % 摧毁半径(m)

        K_omega
        K_attitude
        K_n

        t_plan

        N

        t0 % 起始时间(s)
        dt % 时间步长(s)
        tf % 结束时间(s)

        recode % 记录数据
    end

    methods
        function obj = Missile(t0, dt, tf)
            % 导弹参数
            obj.S_ref = 0.0227;
            obj.L_ref = 1.8;
            obj.L_wing = 0.5;
            obj.R_destroy = 0.5;

            obj.K_omega = 2; % 角速度增益
            obj.K_attitude = -1; % 姿态增益
            obj.K_n = -0.6; % 加速度增益

            obj.t_plan = 51; % 方案飞行时间

            obj.N = 6;

            obj.t0 = t0;
            obj.dt = dt;
            obj.tf = tf;

            length = round((tf - t0) / dt) + 1;
            length = length + 10; % 预留10个点, 提高鲁棒性

            obj.recode.delta_y = zeros(1, length);
            obj.recode.delta_z = zeros(1, length);
            obj.recode.P = zeros(1, length);
            obj.recode.m_c = zeros(1, length);
            obj.recode.alpha = zeros(1, length);
            obj.recode.beta = zeros(1, length);
            obj.recode.gama_V = zeros(1, length);
            obj.recode.X = zeros(1, length);
            obj.recode.Y = zeros(1, length);
            obj.recode.Z = zeros(1, length);
            obj.recode.n_x2 = zeros(1, length);
            obj.recode.n_y2 = zeros(1, length);
            obj.recode.n_z2 = zeros(1, length);
            obj.recode.nu_cmd = zeros(1, length);
            obj.recode.n_y2_cmd = zeros(1, length);
            obj.recode.n_z2_cmd = zeros(1, length);
        end

        function dstates_dt = Missile_Dynamics(obj, t, states, ...
                                                delta_y, delta_z)
            % 导弹6自由度动力学方程组
            % V, theta, phi_V, x, y, z, omega_x, omega_y, omega_z, nu, phi, gama, m
            V = states(1);
            theta = states(2);
            phi_V = states(3);
            omega_x = states(7);
            omega_y = states(8);
            omega_z = states(9);
            nu = states(10);
            gama = states(12);
            m = states(13);

            i = round((t - obj.t0) / obj.dt) + 1;

            g = 9.8;
            [P, m_c] = obj.Missile_Propotion(t);
            [J_x, J_y, J_z] = obj.Missile_Inertia(t);
            [alpha, beta, gama_V] = obj.Missile_Angle(states);
            [X, Y, Z, M_x, M_y, M_z] = obj.Missile_Aerodynamics(t, states, ...
                                                                delta_y, delta_z);
            n_x2 = ( ...
                P * cos(alpha) * cos(beta) ...
                - X) / m / g;
            n_y2 = ( ...
                    P * ( ...
                        sin(alpha) * cos(gama_V) ...
                        + cos(alpha) * sin(beta) * sin(gama_V) ...
                        ) ...
                    + Y * cos(gama_V) ...
                    - Z * sin(gama_V)) / m / g;
            n_z2 = ( ...
                    P * ( ...
                        sin(alpha) * sin(gama_V) ...
                        - cos(alpha) * sin(beta) * cos(gama_V) ...
                        ) ...
                    + Y * sin(gama_V) ...
                    + Z * cos(gama_V)) / m / g;


            dV_dt = ( ...
                        P * cos(alpha) * cos(beta) ...
                        - X ...
                        - m * g * sin(theta) ...
                    ) / m;
            dtheta_dt = ( ...
                            P * ( ...
                                sin(alpha) * cos(gama_V) ...
                                + cos(alpha) * sin(beta) * sin(gama_V) ...
                                ) ...
                            + Y * cos(gama_V) ...
                            - Z * sin(gama_V) ...
                            - m * g * cos(theta) ...
                        ) / m / V;
            dphi_V_dt = - ( ...
                            P * ( ...
                                sin(alpha) * sin(gama_V) ...
                                - cos(alpha) * sin(beta) * cos(gama_V) ...
                                ) ...
                            + Y * sin(gama_V) ...
                            + Z * cos(gama_V) ...
                        ) / m / V / cos(theta);
            domega_x_dt = ( ...
                            M_x ...
                            - (J_z - J_y) * omega_z * omega_y ...
                            ) / J_x;
            domega_y_dt = ( ...
                            M_y ...
                            - (J_x - J_z) * omega_x * omega_z ...
                            ) / J_y;
            domega_z_dt = ( ...
                            M_z ...
                            - (J_y - J_x) * omega_y * omega_x ...
                            ) / J_z;
            
            dx_dt = V * cos(theta) * cos(phi_V);
            dy_dt = V * sin(theta);
            dz_dt = - V * cos(theta) * sin(phi_V);
            dnu_dt = omega_y * sin(gama) + omega_z * cos(gama);
            dphi_dt = (...
                        omega_y * cos(gama) ...
                        - omega_z * sin(gama) ...
                        ) / cos(nu);
            
            dgama_dt = omega_x ...
                        - ( ...
                            omega_y * cos(gama) ...
                            - omega_z * sin(gama) ...
                            ) * tan(nu);
            
            dm_dt = - m_c;

            % 记录数据
            obj.recode.delta_y(i+1) = delta_y;
            obj.recode.delta_z(i+1) = delta_z;
            obj.recode.P(i+1) = P;
            obj.recode.m_c(i+1) = m_c;
            obj.recode.X(i+1) = X;
            obj.recode.Y(i+1) = Y;
            obj.recode.Z(i+1) = Z;
            obj.recode.alpha(i+1) = alpha;
            obj.recode.beta(i+1) = beta;
            obj.recode.gama_V(i+1) = gama_V;
            obj.recode.n_x2(i+1) = n_x2;
            obj.recode.n_y2(i+1) = n_y2;
            obj.recode.n_z2(i+1) = n_z2;

            % V, theta, phi_V, x, y, z, omega_x, omega_y, omega_z, nu, phi, gama, m
            dstates_dt = [dV_dt;
                          dtheta_dt;
                          dphi_V_dt;
                          dx_dt;
                          dy_dt;
                          dz_dt;
                          domega_x_dt;
                          domega_y_dt;
                          domega_z_dt;
                          dnu_dt;
                          dphi_dt;
                          dgama_dt;
                          dm_dt];
        end

        function dstates_dt = Missile_Controler_Dynamics(~, diff_n_y2, diff_n_z2)
            dstates_dt = [diff_n_y2; diff_n_z2];
        end

        function [delta_y, delta_z] = Missile_Control(obj, states, int_n_y2, int_n_z2, nu_cmd, flag)
            phi = states(11);
            omega_y = states(8);
            nu = states(10);
            omega_z = states(9);

            disp(flag);

            if flag == 1
                % 比例导引
                delta_y = obj.K_omega * omega_y ...
                        - obj.K_n * int_n_z2; % 目标角速度
                delta_z = obj.K_omega * omega_z ...
                        + obj.K_n * int_n_y2; % 目标角速度
            else
                % 方案飞行
                delta_y = 0;
                delta_z = obj.K_omega * omega_z ...
                        + obj.K_attitude * (nu_cmd - nu);
            end

            max_delta = deg2rad(30);
            delta_z = max(-max_delta, min(delta_z, max_delta)); % 限制最大值
            delta_y = max(-max_delta, min(delta_y, max_delta)); % 限制最大值
        end

        function [n_y2_cmd, n_z2_cmd, nu_cmd, flag] = Missile_Guidance(obj, t, missile_states, target_states)
            r_rel = target_states(4:6) - missile_states(4:6);
            v_m = missile_states(1);
            theta_m = missile_states(2);
            phi_m = missile_states(3);
            v_t = target_states(1);
            theta_t = target_states(2);
            phi_t = target_states(3);
            V_m_vec = Euler2Vec(v_m, theta_m, phi_m);
            V_t_vec = Euler2Vec(v_t, theta_t, phi_t);
            V_rel = V_t_vec - V_m_vec;

            i = round((t - obj.t0) / obj.dt) + 1;

            if t < obj.t_plan
                flag = 0;
                nu_cmd = (deg2rad(5) - deg2rad(18)) / obj.t_plan * t + deg2rad(18);
                n_y2_cmd = NaN;
                n_z2_cmd = NaN;
            else
                flag = 1;
                nu_cmd = NaN;
                omega = cross(r_rel, V_rel) / norm(r_rel)^2;
                a = - obj.N * norm(V_rel) * cross(V_m_vec, omega) / norm(V_m_vec);
                n_y2_cmd = a(2) / 9.8 + 1;
                n_z2_cmd = a(3) / 9.8;
            end
            obj.recode.nu_cmd(i+1) = nu_cmd;
            obj.recode.n_y2_cmd(i+1) = n_y2_cmd;
            obj.recode.n_z2_cmd(i+1) = n_z2_cmd;

            disp(norm(r_rel));
        end

        function [alpha, beta, gama_V] = Missile_Angle(~, states)
            % V, theta, phi_V, x, y, z, omega_x, omega_y, omega_z, nu, phi, gama, m
            theta = states(2);
            phi_V = states(3);
            nu = states(10);
            phi = states(11);
            gama = states(12);

            beta = asin( ...
                        cos(theta) * ( ...
                            cos(gama) * sin(phi - phi_V) ...
                            + sin(nu) * sin(gama) * cos(phi - phi_V) ...
                            ) ...
                        - sin(theta) * cos(nu) * sin(gama) ...
                        );
            alpha = asin( ...
                        ( ...
                            cos(theta) * ( ...
                                sin(nu) * cos(gama) * cos(phi - phi_V) ...
                                - sin(gama) * sin(phi - phi_V) ...
                                ) ...
                            - sin(theta) * cos(nu) * cos(gama) ...
                        ) / cos(beta) ...
                        );
            gama_V = asin(...
                            ( ...
                                cos(alpha) * sin(beta) * sin(nu) ...
                                - sin(alpha) * sin(beta) * cos(gama) * cos(nu) ...
                                + cos(beta) * sin(gama) * cos(nu) ...
                            ) / cos(theta) ...
                        );
        end

        function [P, m_c] = Missile_Propotion(~, t)
            P = ATM_thrust(t);
            m_c = ATM_mc(t);
        end
        
        function [J_x, J_y, J_z] = Missile_Inertia(~, t)
            J_x = 10; % 忽略滚转
            J_z = ATM_Jz(t); 
            J_y = J_z; % 轴对称导弹
        end

        function [X, Y, Z, M_x, M_y, M_z] = Missile_Aerodynamics(obj, t, states, ...
                                                                delta_y, delta_z)
            % V, theta, phi_V, x, y, z, omega_x, omega_y, omega_z, nu, phi, gama, m
            V = states(1);
            y = states(5);
            omega_z = states(9);
            omega_y = states(8);
            
            [alpha, beta, ~] = obj.Missile_Angle(states);

            xg = ATM_xg(t);
            ma = V / Sound_Speed(y);
            rho = Air_Density(y);
            q = 1/2 * rho * V ^ 2;

            Cx = ATM_Cx(ma, alpha);
            Cy = ATM_Cy(ma, alpha);
            Cz = - ATM_Cy(ma, beta);
            Y = q * obj.S_ref * Cy;
            X = q * obj.S_ref * Cx;
            Z = q * obj.S_ref * Cz;

            mza = ATM_mza(ma, alpha, xg, obj.L_ref);
            mzdz = ATM_mzdz(ma, delta_z);
            mzwz = ATM_mzwz(ma, alpha, xg); % 归一化omega * L /V
            myb = ATM_mza(ma, beta, xg, obj.L_ref);
            mydy = ATM_mzdz(ma, delta_y);
            mywy = ATM_mzwz(ma, beta, xg);
            M_x = 0;
            M_z = (mza + mzdz + mzwz * omega_z * obj.L_wing / V) * q * obj.S_ref * obj.L_wing;
            M_y = (myb + mydy + mywy * omega_y * obj.L_wing / V) * q * obj.S_ref * obj.L_wing;
        end

        function hit = Hit_Ground(~, missile_pos)
            y = missile_pos(2);
            if y < 0
                hit = 1; % 触发 
            else
                hit = 0;
            end
        end

        function hit = Hit_Target(obj, missile_pos, target_pos)
            r_rel = missile_pos - target_pos;
            if norm(r_rel) < obj.R_destroy
                hit = true; % 导弹击中目标
            else
                hit = false; % 导弹未击中目标
            end
        end
    end
end
