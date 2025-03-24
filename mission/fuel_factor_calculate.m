clc; clear; close all;

Is = 2156; % （Ns/kg）发动机比冲
g = 9.801; % (m/s^2)重力加速度
P = 2.2; % 推重比
p0 = 5880; % (N/m^2)翼载

V_t = 420; % (m/s)目标速度
y_t = 15000; % (m)目标高度
y_m0 = 329;
D_t0 = 34200;
q0 = asin((y_t-y_m0)/D_t0); % (rad)弹目夹角

dmu = 0.00001; % 燃料消耗系数
N = round(0.42/dmu);

x_m_list = zeros(1,N);
y_m_list = zeros(1,N);
x_t_list = zeros(1,N);
y_t_list = zeros(1,N);

V = 500; % (m/s)初始速度
theta = deg2rad(26); % (rad)初始弹道倾角
y_m = 329; % (m)初始高度
x_m = 674; % (m)初始位置
alpha = deg2rad(1.5); % (rad)初始攻角
mu = 0;
x_t = 30735;

rho = Air_Density(y_m);
Ma = Sound_Speed(y_m) / V;
Cx = SAM_Cx(Ma, alpha);
Cya = SAM_Cya(Ma, alpha) * alpha;

for i = 1:N
    dVdmu = Is/(1-mu) ...
        - rho*V^2*Cx*Is/(2*P*p0*(1-mu)) ...
        - Is/P*sin(theta);

    dydmu = Is/(P*g)*V*sin(theta);

    dxdmu = Is/(P*g)*V*cos(theta);

    dthetadmu = V_t/y_t ... 
        * (Is/(P * g) + 1/(V^2*sin(theta))*(V*dydmu-y_m*dVdmu)) ...
        / (1 + (cot(q0)-mu*Is*V_t/(P*g*y_t))*cot(theta));

    alpha = (V*dthetadmu + Is*cos(theta)/P)/ ...
        (rho*V^2*Cya*Is/(2*P*p0*(1-mu))+Is/(1-mu));
    
    rho = Air_Density(y_m);
    Ma = Sound_Speed(y_m) / V;
    Cx = SAM_Cx(Ma, alpha);
    Cya = SAM_Cya(Ma, alpha) * alpha;

    V = V + dVdmu * dmu;
    theta = theta + dthetadmu * dmu;
    y_m = y_m + dydmu * dmu;
    x_m = x_m + dxdmu * dmu;
    mu = mu + dmu;
    x_t = x_t - V_t * Is/(P*g)*dmu;
    
    x_m_list(i) = x_m;
    y_m_list(i) = y_m;
    x_t_list(i) = x_t;
    y_t_list(i) = y_t;
    
    r_rel = sqrt((x_m - x_t)^2 + (y_m - y_t)^2);
    disp(r_rel);
    if r_rel < 600
        break;
    end
end

disp(mu);
x_m_list = x_m_list(1:i);
y_m_list = y_m_list(1:i);
x_t_list = x_t_list(1:i);
y_t_list = y_t_list(1:i);

figure;
hold on;
grid on;
axis equal;
title('三点法导引导弹拦截');
xlabel('x(m)');
ylabel('y(m)');
plot(x_m_list, y_m_list, 'b', DisplayName='导弹轨迹');
plot(x_t_list, y_t_list, 'r', DisplayName='目标轨迹');
plot(x_m_list(1), y_m_list(1), 'bo', DisplayName='导弹起始点');
plot(x_t_list(1), y_t_list(1), 'ro', DisplayName='目标起始点');
plot(x_m_list(end), y_m_list(end), 'kx', DisplayName='拦截点');
legend('show');