function [I,R] = find_circle_2d(point1, point2, point3)
    I = zeros(2,1); R = 0;
    x1 = point1(1); x2 = point2(1); x3 = point3(1);
    y1 = point1(2); y2 = point2(2); y3 = point3(2);
    
    ma = (y2-y1)/(x2-x1); mb = (y3-y2)/(x3-x2);
    
    I(1) = (ma*mb*(y1-y3)+mb*(x1+x2)-ma*(x2+x3))/(2*(mb-ma));
    I(2) = -1/ma*(I(1)-(x1+x2)/2)+(y1+y2)/2;
    
    R = sqrt((x1-I(1))^2+(y1-I(2))^2);
end
