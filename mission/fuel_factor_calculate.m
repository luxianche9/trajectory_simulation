clc; clear; close all;

V0 = 500; % (m/s)初始速度
x0 = 674; % (m)初始位置
y0 = 329; % (m)初始高度
alpha0 = deg2rad(1.5); % (rad)初始攻角
theta0 = deg2rad(26); % (rad)初始弹道倾角
Is = 2156; % （Ns/kg）发动机比冲
g = 9.801; % (m/s^2)重力加速度
P_bar = 2.2; % 推重比
p0 = 5880; % (N/m^2)翼载

% 三点法导引，目标等高迎头飞行
V_t = -420; % (m/s)目标速度
y_t = 15000; % (m)目标高度
D_t0 = 34200; % (m)弹目斜距

q0 = asin((y_t-y0)/D_t0); % (rad)弹目夹角

dmu = 0.001; % 燃料消耗系数
N = 1/dmu;

V = V0;
theta = theta0;
y = y0;
x = x0;
rho = AtmosphericDensity(y);
Ma = LocalSoundSpeed(y) / V;
alpha = alpha0;
mu = 1;
[Cx, Cy_alpha] = AeroCoeffInterp(Ma, alpha);

x_list = zeros(1,N);
y_list = zeros(1,N);
V_list = zeros(1,N);
theta_list = zeros(1,N);

for i = 1:N
    mu = mu - dmu;

    Ma = LocalSoundSpeed(y) / V;
    rho = AtmosphericDensity(y);

    dydmu = Is/(P_bar*g)*V*sin(theta);

    dxdmu = Is/(P_bar*g)*V*cos(theta);

    dVdmu = Is/(1-mu) ...
            - rho*V^2*Cy_alpha*Is/(2*P_bar*p0*(1-mu)) ...
            - Is/P_bar*sin(theta);

    dthetadmu = V_t/y_t * (Is/(P_bar * g) + 1/(V^2*sin(theta))*(V*dydmu-y*dVdmu)) ...
                /(1 + (cot(q0)-mu*Is*V_t/(P_bar*g*y_t))*cot(theta));

    alpha = (V*dthetadmu + Is*cos(theta))/ ...
        (rho*V^2*Cy_alpha*Is/(2*P_bar*p0*(1-mu))+Is/(1-mu));
    
    [Cx, Cy_alpha] = AeroCoeffInterp(Ma, alpha);

    y = y + dydmu;
    x = x + dxdmu;
    V = V + dVdmu;
    theta = theta + dthetadmu;
    
    x_list(i) = x;
    y_list(i) = y;
    V_list(i) = V;
    theta_list(i) = theta;
    
end
