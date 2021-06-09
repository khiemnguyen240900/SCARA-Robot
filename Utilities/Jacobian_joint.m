function j = Jacobian_joint(robot,joint)
    framei_1 = robot.get('frame0');
    index = joint.get('index');
    while ~(framei_1.get('index') == index - 1)
        framei_1 = framei_1.Next;
    end
%     zi_1 = framei_1.get('z');
    zi_1 = [0;0;1];
    joint_type = joint.get('type');
    switch joint_type
        case 'prismatic'
            j = [zi_1;0;0;0];
        case 'revolute'
            j = [cross(zi_1,robot.get('ef_p') - framei_1.get('p'));zi_1];
        otherwise
            warning('Something went wrong!!!');
    end
end
    