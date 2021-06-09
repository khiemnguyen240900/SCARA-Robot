classdef C_Scara < handle
    properties (Access = public)
        model;
    end
    properties (Access = private)
        qr; %q respone
        q_;q_dot_; q_dot; q_2dot;
        q; q_next; q_mov; 
        ef_p; ef_o; ef_p_mov;
        inverseParameters;
        a_max; mov_cycle; cycle; per_v;
        isInitialized; isMoving; isUsingJacobian; isPlanned; isUsingDynamics;
        s_,s_mov;s_dot_mov;s_2dot_mov;%including s,s_dot,s_2dot
        link0 C_Link; link1 C_Link; link2 C_Link; link3 C_Link; link4 C_Link;     
        joint1 C_Joint; joint2 C_Joint; joint3 C_Joint; joint4 C_Joint;
        frame0 C_Frame; frame1 C_Frame; frame2 C_Frame; frame3 C_Frame; frame4 C_Frame;
        timer timer; Ts;
        model_status;
        J; J_main; %Jacobian
    end
    
    methods (Access = public)
        function obj = C_Scara(DH_params,Ts)
            %Timer
            obj.Ts = Ts;
            obj.timer = timer;
            %Parameters
            obj.a_max = 0.05; obj.per_v = 0.9;
            obj.q = zeros(4,1); obj.q_next = zeros(4,1); obj.qr = zeros(4,1);
            obj.ef_p = zeros(3,1);
            obj.ef_o = zeros(3,1);
            obj.s_ = zeros(3,1);
            %{
            DH_parameter = [a0 alpha0 d0 theta0; %link0
                            a1 alpha1 d1 theta1; %link1
                            a2 alpha2 d2 theta2; %link2
                            a3 alpha3 d3 theta3; %link3
                            a4 alpha4 d4 theta4] %link4
            %}
            %Links: C_Link(index,DH_params,_previousLink)
            obj.link0 = C_Link(0,DH_params(1,:)); %base
            obj.link1 = C_Link(1,DH_params(2,:),obj.link0);
            obj.link2 = C_Link(2,DH_params(3,:),obj.link1);
            obj.link3 = C_Link(3,DH_params(4,:),obj.link2);
            obj.link4 = C_Link(4,DH_params(5,:),obj.link3);
            %Joints: C_Joint(index,type,parentLink,_previousJoint)
            obj.joint1 = C_Joint(1,'revolute',obj.link0);
            obj.joint2 = C_Joint(2,'revolute',obj.link1,obj.joint1);
            obj.joint3 = C_Joint(3,'prismatic',obj.link2,obj.joint2);
            obj.joint4 = C_Joint(4,'revolute',obj.link3,obj.joint3);
            %Frames: C_Frame(index, link, _joint, _frame0, _previousFrame)
            obj.frame0 = C_Frame(0, obj.link0);
            obj.frame1 = C_Frame(1, obj.link1, obj.joint1, obj.frame0, obj.frame0);
            obj.frame2 = C_Frame(2, obj.link2, obj.joint2, obj.frame0, obj.frame1);
            obj.frame3 = C_Frame(3, obj.link3, obj.joint3, obj.frame0, obj.frame2);
            obj.frame4 = C_Frame(4, obj.link4, obj.joint4, obj.frame0, obj.frame3);
            %Jacobian
            obj.isUsingJacobian = false;
            obj.isUsingDynamics = false;
            obj.J = zeros(6,4);
            obj.J_main = obj.J(1:2,:);
            %Initilization Flag
            obj.isInitialized = true;
            obj.isPlanned = false;
            obj.isMoving = false;
            %Model
            obj.model = 'scara_model';
        end
        
        function initialize(obj)
            if obj.isInitialized
                obj.joint1.tformAT();
                obj.joint2.tformAT();
                obj.joint3.tformAT();
                obj.joint4.tformAT();
                obj.frame0.update();
                obj.frame1.update();
                obj.frame2.update();
                obj.frame3.update();
                obj.frame4.update();
                obj.q = [obj.joint1.get('q');obj.joint2.get('q');obj.joint3.get('q');obj.joint4.get('q')]; obj.q_next = obj.q;                
                obj.q_dot = zeros(4,1);
                obj.q_2dot = zeros(4,1);
                obj.inverseParameters = [obj.link1.get('a'); obj.link2.get('a'); obj.link1.get('d'); obj.link4.get('d')];
                obj.ef_p = obj.frame4.get('p');
                obj.ef_o = obj.frame4.get('o');
                obj.timer.TimerFcn = @(~, ~)obj.timerHandle;
                obj.timer.Period = obj.Ts;
                obj.timer.ExecutionMode = 'fixedRate';
                open_system(obj.model);
                pause(1);
                obj.isInitialized = true;
            else
                warning('Scara robot has been initialized!');
            end
        end
        
        function startRobot(obj)
            obj.model_status = get_param(obj.model,'SimulationStatus');
            if obj.model_status == "running"
                start(obj.timer);
            else
                warning('The robot is not running');
            end
        end
        
        function forwardKinematics(obj,q_desired)
            obj.model_status = get_param(obj.model,'SimulationStatus');
            if obj.model_status == "running"
                obj.update_all();
                obj.q_next = q_desired;
                % Update model
                obj.update_model();
                obj.update_all();
            else
                warning('The robot is not running');
            end
        end
        
        function inverseKinematics(obj,ef_desired_pose)
            obj.model_status = get_param(obj.model,'SimulationStatus');
            if obj.model_status == "running"
                obj.update_all();
                obj.q_next = inverse_kinematics_scara(ef_desired_pose,obj.inverseParameters);
                % Update model
                obj.update_model();
                obj.update_all();
            else
                warning('The robot is not running');
            end
        end
        
        function movLinear(obj, ef_desired_p)
            obj.model_status = get_param(obj.model,'SimulationStatus');
            if obj.model_status == "running"
                if (ef_desired_p == obj.ef_p)
                    warning('Robot has already been at this position');
                else
                    obj.update_all();
                    [s_max, path] = obj.path_planning_linear(ef_desired_p);
                    [s, s_dot, s_2dot] = obj.trajectory_planning(s_max);
                    obj.s_mov = s; obj.s_dot_mov = s_dot; obj.s_2dot_mov = s_2dot;
                    obj.cal_ef_p_mov_linear(s_max, path);
                    obj.mov_inverseKinematics();
                    disp('Planning linear: success');
                    obj.isPlanned = true;
                end
            else
                warning('The robot is not running');
            end
        end
        
        function movCircular2D(obj, ef_desired_p_1, ef_desired_p_2)
            obj.model_status = get_param(obj.model,'SimulationStatus');
            if obj.model_status == "running"
                if (ef_desired_p_2 == obj.ef_p)
                    warning('Robot has already been at this position');
                else
                    obj.update_all();
                    [I, R, s_max, direction] = obj.path_planning_circular_2D(ef_desired_p_1, ef_desired_p_2);
                    [s, s_dot, s_2dot] = obj.trajectory_planning(s_max);
                    obj.s_mov = s; obj.s_dot_mov = s_dot; obj.s_2dot_mov = s_2dot;
                    obj.cal_ef_p_mov_circular_2D(direction, R, I);
                    obj.mov_inverseKinematics();
                    disp('Planning circular 2D: Success');
                    obj.isPlanned = true;
                end
            else
                warning('The robot is not running');
            end
        end
        
        function result = get(obj,params)
            obj.update_all();
            switch params
                case 'update'
                    result = 'updated';
                case 'q'
                    result = obj.q;
                case 'ef_p'
                    result = obj.ef_p;
                case 'ef_p_mov'
                    result = obj.ef_p_mov;
                case 'ef_o'
                    result = obj.ef_o;
                case 'ef_pose'
                    result = [obj.ef_p;obj.ef_o];
                case 'model'
                    result = obj.model;
                case 'link'
                    result = [obj.link0; obj.link1; obj.link2; obj.link3; obj.link4];
                case 'joint'
                    result = [obj.joint1; obj.joint2; obj.joint3; obj.joint4];
                case 'frame'
                    result = [obj.frame0; obj.frame1; obj.frame2; obj.frame3; obj.frame4];
                case 'model_status'
                    obj.model_status = get_param(obj.model,'SimulationStatus');
                    result = obj.model_status;
                case 'mov_cycle'
                    result = obj.mov_cycle;
                case 'J'
                    result = obj.J;
                case 'J_main'
                    result = obj.J_main;
                case 'frame0'
                    result = obj.frame0;
                case 'isMoving'
                    result = obj.isMoving;
                case 's_'
                    result = obj.s_;
                case 'cycle'
                    result = obj.cycle;
                case 'q_dot'
                    if obj.isMoving
                        result = obj.q_dot;
                    else
                        result = zeros(4,1);
                    end
                case 'q_2dot'
                    if obj.isMoving
                        result = obj.q_2dot;
                    else
                        result = zeros(4,1);
                    end
                case 'qr'
                    if (obj.isMoving == true)&&(obj.isUsingDynamics == true)
                        result = obj.qr;
                    else
                        result = zeros(4,1);
                    end
                case 'isUsingDynamics'
                    result = obj.isUsingDynamics;
                otherwise
                    error('Wrong number of input arguments.');                
            end
        end
        
        function delete(obj)
            stop(obj.timer);
            set_param(obj.model, 'SimulationCommand', 'stop');
        end
        
        function mov(obj)
            obj.model_status = get_param(obj.model,'SimulationStatus');
            if obj.model_status == "running"
                if obj.isPlanned
                    obj.q_ = zeros(4,1);
                    obj.q_dot_ = zeros(4,1);
                    obj.isMoving = true;
                    obj.isPlanned = false;
                    obj.cycle = 1;
                else
                    warning('Not planned yet');
                end
            else
                warning('The robot is not running');
            end
        end
        
        function stopMov(obj)
            obj.isMoving = false;
            obj.isPlanned = false;
        end
        
        function useJacobian(obj) 
            obj.isUsingJacobian = true; 
        end
        
        function stopJacobian(obj)
            obj.isUsingJacobian = false;
        end
        
        function useDynamics(obj) 
            obj.isUsingDynamics = true; 
        end
        
        function stopDynamics(obj) 
            obj.isUsingDynamics = false; 
        end
        
        function stopTimer(obj)
            stop(obj.timer);
        end
        
        function set(obj,params,value)
            switch params
                case 'a_max'
                    obj.a_max = value;
                case 'per_v'
                    obj.per_v = value;
                otherwise
                    error('Wrong number of input arguments.');  
            end
        end
    end
    
    methods (Access = private)
        function update_all(obj)
            obj.joint1.update(obj.q(1)); obj.joint2.update(obj.q(2)); obj.joint3.update(obj.q(3)); obj.joint4.update(obj.q(4));
            obj.frame1.update();         obj.frame2.update();         obj.frame3.update();         obj.frame4.update();         
            obj.ef_p = obj.frame4.get('p');                           obj.ef_o = obj.frame4.get('o');
        end
        
        function update_model(obj)
            if obj.isMoving
                if obj.cycle >= obj.mov_cycle
                    obj.isMoving = false;
                end
                obj.s_(1) = obj.s_mov(obj.cycle); obj.s_(2) = obj.s_dot_mov(obj.cycle); obj.s_(3) = obj.s_2dot_mov(obj.cycle);
                obj.q_next = obj.q_mov(:,obj.cycle);
                obj.cycle = obj.cycle + 1;
                obj.cal_q_();
            end
            if obj.isUsingJacobian
                obj.Jacobian();                
            end
            obj.q = obj.q_next;
            obj.model_status = get_param(obj.model,'SimulationStatus');
            if obj.isUsingDynamics
                set_param('scara_model/qd1','Gain',num2str(obj.q(1)));
                set_param('scara_model/qd2','Gain',num2str(obj.q(2)));
                set_param('scara_model/qd3','Gain',num2str(obj.q(3)));
                set_param('scara_model/qd4','Gain',num2str(obj.q(4)));
                q1_rto = get_param('scara_model/qr1','RuntimeObject'); q2_rto = get_param('scara_model/qr2','RuntimeObject'); q3_rto = get_param('scara_model/qr3','RuntimeObject'); q4_rto = get_param('scara_model/qr4','RuntimeObject');
                obj.qr(1) = q1_rto.OutputPort(1).Data(1); obj.qr(2) = q2_rto.OutputPort(1).Data(1); obj.qr(3) = q3_rto.OutputPort(1).Data(1); obj.qr(4) = q4_rto.OutputPort(1).Data(1);
            else
                obj.qr = obj.q;
            end
            set_param('scara_model/q1','Gain',num2str(obj.qr(1)));
            set_param('scara_model/q2','Gain',num2str(obj.qr(2)));
            set_param('scara_model/q3','Gain',num2str(obj.qr(3)));
            set_param('scara_model/q4','Gain',num2str(obj.qr(4)));
        end
        
        function timerHandle(obj)
            obj.model_status = get_param(obj.model,'SimulationStatus');
            if obj.model_status == "running"
                update_model(obj);
            else
                error('Scara robot simulation is not running!');
            end
        end
        
        function [s, s_dot, s_2dot] = trajectory_planning(obj,s_max)
            [s, s_dot, s_2dot, obj.mov_cycle] = trajectory_planning(s_max, obj.a_max, obj.per_v, obj.Ts);
        end
        
        function [s_max, path] = path_planning_linear(obj, ef_desired_p)
            [s_max, path] = path_planning_linear(obj.ef_p, ef_desired_p);
        end
        
        function [I, R, s_max, direction] = path_planning_circular_2D(obj, ef_desired_p_1, ef_desired_p_2)
            [I, R, s_max, direction] = path_planning_circular_2D(obj.ef_p, ef_desired_p_1, ef_desired_p_2);
        end
        
        function cal_ef_p_mov_linear(obj,s_max, path)
            obj.ef_p_mov = zeros(3,obj.mov_cycle);
            for i = 1 : obj.mov_cycle
                obj.ef_p_mov(:,i) = obj.ef_p + (obj.s_mov(i)/s_max)*path;
            end
        end
        
        function cal_ef_p_mov_circular_2D(obj,direction, R, I)
            z_ = [0;0;-1]; x_ = (obj.ef_p-I)/norm(obj.ef_p-I); y_ = cross(z_,x_);
            frameA = [1 0 0; 0 1 0; 0 0 1]; frameB = [x_, y_, z_];
            rotation_matrix = find_rotation_matrix(frameA,frameB);
            obj.ef_p_mov = zeros(3,obj.mov_cycle);
            for i = 1 : obj.mov_cycle
                obj.ef_p_mov(:,i) = I + rotation_matrix*[R*cos(direction*obj.s_mov(i)/R); R*sin(direction*obj.s_mov(i)/R);0];
            end
        end
        
        function mov_inverseKinematics(obj)
            obj.q_mov = zeros(4,obj.mov_cycle);
            for i = 1:obj.mov_cycle
                obj.q_mov(:,i) = inverse_kinematics_scara([obj.ef_p_mov(:,i);obj.ef_o],obj.inverseParameters);
            end
        end
        
        function Jacobian(obj)
            obj.J(:,1) = Jacobian_joint(obj,obj.joint1);
            obj.J(:,2) = Jacobian_joint(obj,obj.joint2);
            obj.J(:,3) = Jacobian_joint(obj,obj.joint3);
            obj.J(:,4) = Jacobian_joint(obj,obj.joint4);
            obj.J_main = obj.J(1:2,:);
        end
        
        function cal_q_(obj)
            if obj.q_ ~= zeros(4,1)
                obj.q_dot = (obj.q - obj.q_)/obj.Ts;
                obj.q_2dot = (obj.q_dot - obj.q_dot_)/obj.Ts;
            end
            obj.q_ = obj.q;
            obj.q_dot_ = obj.q_dot;
        end
            
    end
    
end