function dydt = Target_Dynamics_tank(~, y)
    % 目标直线运动
    % vx vz x z
    dydt(1) = 0;
    dydt(2) = 0;
    dydt(3) = y(1);
    dydt(4) = y(2);
end
