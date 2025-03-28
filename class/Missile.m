classdef Missile
    properties
        % 导弹参数
        S_ref % 特征面积(m^2)
        L_ref % 特征长度(m)
        L_wing % 翼展(m)
        % 仿真过程状态记录
        states_list
        t0
        dt
        tf
    end

    methods
        function obj = Missile(t0, dt, tf)
            % 导弹参数
            obj.S_ref = 0.0227;
            obj.L_ref = 1.8;
            obj.L_wing = 0.5;
            obj.t0 = t0;
            obj.dt = dt;
            obj.tf = tf;
        
            obj.states_list = zeros(27, floor((tf - t0) / dt) + 1);
        end

        % function obj = Record_Data(obj, t, states, alpha, beta, gama_V, P, m_c, J_x, J_y, J_z, X, Y, Z, M_x, M_y, M_z)
        %     i = floor(t / obj.dt) + 1;
        %     % V, theta, phi_V, x, y, z, omega_x, omega_y, omega_z, nu, phi, gama, m
        %     obj.states_list(1:13, i) = states;
        %     obj.states_list(14:16, i) = [alpha, beta, gama_V];
        %     obj.states_list(17:18, i) = [P, m_c];
        %     obj.states_list(19:21, i) = [J_x, J_y, J_z];
        %     obj.states_list(22:27, i) = [X, Y, Z, M_x, M_y, M_z];
        % end

        function [dstates_dt, obj] = Missile_Dynamics(obj, t, states, ...
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

            g = 9.8;

            [J_x, J_y, J_z] = obj.Missile_Inertia(t);

            [alpha, beta, gama_V] = obj.Missile_Angle(states);

            [X, Y, Z, M_x, M_y, M_z] = obj.Missile_Aerodynamics(t, states, ...
                                                                delta_y, delta_z);

            [P, m_c] = obj.Missile_Propotion(t);
            
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

        function [value, isterminal, direction] = Hit_Ground(~, ~, states)
            isterminal = 1; % 终止仿真
            direction = 0;
            y = states(5);
            if y < 0
                value = 0; % 触发 
            else
                value = 1;
            end
        end
    end
end

