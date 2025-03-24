function flag = hit_ground(~ , y)
    h = y(5);
    
    if (h <= 0)
        flag = 1;
    else
        flag = 0;
    end
end