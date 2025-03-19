x = 1;

T = 1;

f = @(t, y) First_Order_Process(t, y, x, T);

[t, y] = ode_EPC(0, 0.01, 10, 0, f);

plot(t, y);