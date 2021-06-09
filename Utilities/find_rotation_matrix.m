function [rotation_matrix] = find_rotation_matrix(frameA,frameB)
    rotation_matrix = frameB/frameA;
end

