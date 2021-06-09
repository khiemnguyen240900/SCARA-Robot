function q_next = inverse_kinematics_scara(ef_desired_pose,inverseParameters)
    q_next = zeros(4,1);
    
    a1 = inverseParameters(1); a2 = inverseParameters(2); d1 = inverseParameters(3); d4 = inverseParameters(4);
    Pwx = ef_desired_pose(1); Pwy = ef_desired_pose(2);
    
    % Calculate (sin cos) value
    c2 = (Pwx^2 + Pwy^2 - a1^2 - a2^2)/(2*a1*a2);
    q_next(2) = + acos(c2);
    alpha = atan2(Pwy,Pwx);
    beta = acos((Pwx^2 + Pwy^2 + a1^2 - a2^2)/(2*a1*sqrt(Pwx^2 + Pwy^2)));
    q_next(1) = alpha - beta;
    q_next(3) = ef_desired_pose(3) - d1 - d4;
    q_next(4) = ef_desired_pose(6) - q_next(1) - q_next(2);
end