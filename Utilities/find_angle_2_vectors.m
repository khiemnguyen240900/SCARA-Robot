function [angle] = find_angle_2_vectors(v1,v2,n)
    x = cross(v1,v2);
    c = sign(dot(x,n)) * norm(x);
    angle = atan2(c,dot(v1,v2));
end

