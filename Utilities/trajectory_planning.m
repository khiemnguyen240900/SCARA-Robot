function [s, s_dot, s_2dot, mov_cycle] = trajectory_planning(s_max, a_max, per_v, Ts)
    % [s, s_dot, s_2dot, mov_cycle] = trajectory_planning(s_max, a_max, per_v, Ts)
    v_max = per_v * sqrt(s_max * a_max);
    disp(per_v);
    % 3 stage: 0-t1, t1-t2, t2-t3
    t1 = v_max/a_max;
    t2 = (s_max - v_max*t1)/v_max + t1;
    t3 = t1 + t2; %t3 = t1+(t2-t1)+(t3-t2) because (t3-t2)=t1   
    t = 0:Ts:t3;
    t(length(t)+1) = t3;
    len = length(t);
    mov_cycle = len;
    s = zeros(len,1); s_dot = zeros(len,1); s_2dot = zeros(len,1);
    for i = 1:len
       %stage 1
       if t(i) < t1
           s(i) = a_max * t(i)^2 / 2;
           s_dot(i) = a_max * t(i);
           s_2dot(i) = a_max;
       %stage 2
       elseif t(i) < t2
           s(i) = a_max * t1^2 / 2 + v_max * (t(i) - t1);
           s_dot(i) = v_max;
           s_2dot(i) = 0;
       %stage 3
       else
           s(i) = a_max * t1^2 / 2 + v_max * (t2 - t1) + (v_max * (t(i) - t2) - a_max * (t(i) - t2)^2 / 2);
           s_dot(i) = v_max - a_max * (t(i) - t2);
           s_2dot(i) = - a_max;
       end
    end
end