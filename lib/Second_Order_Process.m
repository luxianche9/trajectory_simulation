function dydt = Second_Order_Process(t, y, x, T, zeta)
    y_old = y(1);
    dy_old = y(2);
    
    ddy = x / T^2 - y_old / T^2 - 2 * zeta / T * dy_old;
    
    dydt = [dy_old; ddy];
end