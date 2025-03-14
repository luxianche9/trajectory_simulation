clear;
clc;
close all;

figure;
hold on;
grid on;
title('导弹与目标弹道仿真');
xlabel('x(m)');
ylabel('y(m)');
[t, t_x, t_y, m_x, m_y, a_m, a_c, r] = ...
    trajectory_pn('PPN', 3, 'circle',2);
plot(t_x,t_y,'r--',DisplayName='目标',LineWidth=2);
plot(m_x,m_y,'b-',DisplayName='导弹',LineWidth=2);
legend();
hold off

figure;
hold on
grid on;
title('导弹加速度')
xlabel('t(s)')
ylabel('a_m(m/s^2)')
for N = 3:1:6
    [t, t_x, t_y, m_x, m_y, a_m, a_c, r] = ...
        trajectory_pn('TPN', N, 'circle',2);
    plot(t,a_m,'LineWidth',1,'DisplayName',['N=' num2str(N)]);
    min(r)
end
legend;
hold off;