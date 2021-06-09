classdef C_Frame < dlnode
    properties (Access = public)
        index;
        p;
        o;
        joint C_Joint;
        link C_Link;
        frame0 C_Frame;
    end
    
    methods (Access = public)
        function obj = C_Frame(index, link, joint, frame0, previousFrame)
            obj.p = zeros(3,1);
            obj.o = zeros(3,1);
            if (nargin == 2)||(nargin == 3)||(nargin == 5)                
                obj.index = index;
                obj.link = link;
                if (nargin >= 3)
                    obj.joint = joint;
                end
                if (nargin == 5)
                    obj.insertAfter(previousFrame);
                    obj.frame0 = frame0;
                end
            else
                error('Wrong number of input arguments.')
            end
        end
        
        function update(obj)
            if obj.index == 0 %frame0
                a = obj.link.get('a');
                alpha = obj.link.get('alpha');
                d = obj.link.get('d');
                theta = obj.link.get('theta');
                T_ = tform(a, alpha, d, theta);
                p_ = [0;0;0]; %frame base
            else
                T_ = obj.joint.get('T');
                p_ = obj.frame0.get('p');
            end
            % Position
            extended_position = [p_;1];
            temp_position     = T_*extended_position;
            obj.p = temp_position(1:3);
            % Orientation
            obj.o(1) = atan2(T_(3,2),T_(3,3));
            obj.o(2) = atan2(-T_(3,1),sqrt(T_(3,2)^2+T_(3,3)^2));
            obj.o(3) = atan2(T_(2,1),T_(1,1));
        end
        
        function result = get(obj,params)
            switch params
                case 'index'
                    result = obj.index;
                case 'p'
                    result = obj.p;
                case 'o'
                    result = obj.o;
                case 'joint'
                    result = obj.joint;
                case 'link'
                    result = obj.link;
                otherwise
                    error('Wrong number of input arguments.');   
            end
        end
    end
end