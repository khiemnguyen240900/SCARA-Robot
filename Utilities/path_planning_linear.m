function [s_max, linear_path] = path_planning_linear(ef_p, ef_desired_p)
% [s_max, linear_path] = path_planning_linear(ef_p, ef_desired_p)
    %% Calculate the position vector (linear)
    linear_path = ef_desired_p - ef_p;
    s_max = norm(linear_path); %path length = distance
end