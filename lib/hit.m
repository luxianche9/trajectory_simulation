function flag = hit(~ , y)
    x_m = y(4);
    y_m = y(5);
    z_m = y(6);
    x_t = y(11);
    y_t = y(12);
    z_t = y(13);

    r_rel = [x_t - x_m;
             y_t - y_m;
             z_t - z_m];

    % fprintf("r_rel = %.2f\n", norm(r_rel));
    
    if (norm(r_rel) < 1)
        flag = 1;
    else
        flag = 0;
    end
end