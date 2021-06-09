function [I, R, s_max, direction] = path_planning_circular_2D(ef_p, ef_desired_p_1,ef_desired_p_2)
    I = zeros(3,1);
    point1 = ef_p(1:2); point2 = ef_desired_p_1(1:2); point3 = ef_desired_p_2(1:2); 
    [I(1:2), R] = find_circle_2d(point1, point2, point3);
    
    u = point1 - I(1:2); v = point3 - I(1:2);
    angle = find_angle_2_vectors([u;0],[v;0],[0;0;-1]);
    
    s_max = angle*R;
    direction = sign(s_max);
    s_max = abs(s_max);
    I(3) = ef_p(3);
end

