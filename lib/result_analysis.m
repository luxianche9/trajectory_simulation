function result_analysis(missile, t, y)
    y = y';
    length_t = length(t);
    
    %% 提取数据
    V_m = y(:, 1); theta = y(:, 2); phi_V = y(:, 3);
    x_m = y(:, 4); y_m = y(:, 5); z_m = y(:, 6);
    omega_x = y(:, 7); omega_y = y(:, 8); omega_z = y(:, 9);
    nu = y(:, 10); phi = y(:, 11); gama = y(:, 12);

    n_x2 = missile.recode.n_x2(1:length_t);
    n_y2 = missile.recode.n_y2(1:length_t);
    n_z2 = missile.recode.n_z2(1:length_t);

    x_t = y(:, 17); y_t = y(:, 18); z_t = y(:, 19);

    delta_y = missile.recode.delta_y(1:length_t);
    delta_z = missile.recode.delta_z(1:length_t);
    nu_cmd = missile.recode.nu_cmd(1:length_t);
    n_y2_cmd = missile.recode.n_y2_cmd(1:length_t);
    n_z2_cmd = missile.recode.n_z2_cmd(1:length_t);

    t_plan = missile.t_plan;


    %% 1. 三维轨迹图
    figure;
    pos_m = [x_m'; y_m'; z_m'];
    pos_t = [x_t'; y_t'; z_t'];
    L = [1 0 0; 0 0 -1; 0 1 0];
    pos_m = L * pos_m;
    pos_t = L * pos_t;

    idx_plan = find(t < t_plan);
    idx_guidance = find(t >= t_plan);

    hold on;
    plot3(pos_m(1, idx_plan), pos_m(2, idx_plan), pos_m(3, idx_plan), 'b-', "DisplayName", '方案飞行段', 'LineWidth', 1.8);
    plot3(pos_m(1, idx_guidance), pos_m(2, idx_guidance), pos_m(3, idx_guidance), 'r-', "DisplayName", '比例导引段', 'LineWidth', 1.8);
    plot3(pos_t(1, :), pos_t(2,:), pos_t(3,:), 'k--', "DisplayName", '目标轨迹', 'LineWidth', 1.8);
    plot3(pos_m(1,end), pos_m(2,end), pos_m(3,end), 'ko', 'MarkerSize', 6, 'MarkerFaceColor', 'y', 'DisplayName', '打击点');
    hold off;
    xlabel('x_m (m)'); ylabel('z_m (m)'); zlabel('y_m (m)'); title('导弹与目标轨迹');
    grid on; view(3); legend('show'); axis equal;

    %% 2. 角速度 (1x3)
    figure;
    subplot(1,3,1); plot(t, omega_x, 'LineWidth', 1.8); grid on; xlabel('时间 (s)'); ylabel('\omega_x'); title('角速度 \omega_x');
    subplot(1,3,2); plot(t, omega_y, 'LineWidth', 1.8); grid on; xlabel('时间 (s)'); ylabel('\omega_y'); title('角速度 \omega_y');
    subplot(1,3,3); plot(t, omega_z, 'LineWidth', 1.8); grid on; xlabel('时间 (s)'); ylabel('\omega_z'); title('角速度 \omega_z');

    %% 3. 姿态角 (1x3)
    figure;
    subplot(1,3,1); plot(t, rad2deg(gama), 'LineWidth', 1.8); grid on; xlabel('时间 (s)'); ylabel('\gama (°)'); title('滚转角 \gama');
    subplot(1,3,2); plot(t, rad2deg(phi), 'LineWidth', 1.8); grid on; xlabel('时间 (s)'); ylabel('\phi (°)'); title('偏航角 \phi');
    subplot(1,3,3); plot(t, rad2deg(nu), 'LineWidth', 1.8); hold on; plot(t, rad2deg(nu_cmd), '--', 'LineWidth', 1.5); grid on;
    xlabel('时间 (s)'); ylabel('\nu (°)'); title('俯仰角 \nu'); legend('实际', '指令');

    %% 4. 舵偏角 (1x2)
    figure;
    subplot(1,2,1); plot(t, rad2deg(delta_y), 'LineWidth', 1.8); grid on; xlabel('时间 (s)'); ylabel('\delta_y (°)'); title('俯仰舵');
    subplot(1,2,2); plot(t, rad2deg(delta_z), 'LineWidth', 1.8); grid on; xlabel('时间 (s)'); ylabel('\delta_z (°)'); title('偏航舵');

    %% 5. 过载 (1x3)
    figure;
    subplot(1,3,1); plot(t, n_x2, 'LineWidth', 1.8); grid on; xlabel('时间 (s)'); ylabel('n_{x2}'); title('纵向过载');
    subplot(1,3,2); plot(t, n_y2, 'LineWidth', 1.8); hold on; plot(t, n_y2_cmd, '--', 'LineWidth', 1.5); grid on;
    xlabel('时间 (s)'); ylabel('n_{y2}'); title('俯仰向过载'); legend('实际', '指令');
    subplot(1,3,3); plot(t, n_z2, 'LineWidth', 1.8); hold on; plot(t, n_z2_cmd, '--', 'LineWidth', 1.5); grid on;
    xlabel('时间 (s)'); ylabel('n_{z2}'); title('偏航向过载'); legend('实际', '指令');

    %% 6. 速度 (1x3)
    figure;
    subplot(1,3,1); plot(t, rad2deg(phi_V), 'LineWidth', 1.8); grid on; xlabel('时间 (s)'); ylabel('\phi_v (°)'); title('速度偏角 \phi_v');
    subplot(1,3,2); plot(t, rad2deg(theta), 'LineWidth', 1.8); grid on; xlabel('时间 (s)'); ylabel('\theta (°)'); title('速度倾角 \theta');
    subplot(1,3,3); plot(t, V_m, 'LineWidth', 1.8); grid on; xlabel('时间 (s)'); ylabel('V_m (m/s)'); title('导弹速度');
end
