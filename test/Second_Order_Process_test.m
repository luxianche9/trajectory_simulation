clear;

x = 1;

T = 1;

zeta = 0.3;

y0 = [0, 0];

f = @(t, y) Second_Order_Process(t, y, x, T, zeta);

[t, y] = ode_EPC(0, 0.01, 10, y0, f);

y = y(1,:);

plot(t, y);