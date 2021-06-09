function rad = d2r(deg)
    rad = deg/180*pi;
    if (rad > 3.14159265359) 
        rad = 3.14159265359;
    end
    if (rad < -3.14159265359) 
        rad = - 3.14159265359;
    end
end
