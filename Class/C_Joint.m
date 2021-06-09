classdef C_Joint < dlnode
    properties (Access = private)
        index;
        type;
        parentLink C_Link; 
        childLink C_Link; 
        q;
        A = zeros(4); 
        T = zeros(4);
    end
    
    methods (Access = public)
        function obj = C_Joint(index,type,parentLink,previousJoint)
            if (nargin == 3)||(nargin == 4)
                obj.index = index;
                obj.type = type;
                obj.parentLink = parentLink;
                obj.childLink = parentLink.Next;
                % jointObj.A = tformA(jointObj.childLink); %Cannot use this
                if nargin == 4
                    obj.insertAfter(previousJoint);
                end
            else
                error('Wrong number of input arguments.')
            end
            switch type
                case 'fixed'
                case 'revolute'
                    obj.q = obj.childLink.get('theta');
                case 'prismatic'
                    obj.q = obj.childLink.get('d');
            end
        end
        
        function tformAT(obj)
            a     = obj.childLink.get('a');
            alpha = obj.childLink.get('alpha');
            d     = obj.childLink.get('d');
            theta = obj.childLink.get('theta');            
            % A
            obj.A =  tform(a,alpha,d,theta);
            % T
            if isempty(obj.Prev)
                obj.T = obj.A;
            else
                obj.T = obj.Prev.T * obj.A;
            end
        end
        
        function result = get(obj,params)
            switch params
                case 'index'
                    result = obj.index;
                case 'type'
                    result = obj.type;
                case 'q'
                    result = obj.q;
                case 'A'
                    result = obj.A;
                case 'T'
                    result = obj.T;
                otherwise
                    error('Wrong number of input arguments.');                
            end
        end
        
        function update(obj, value)
            obj.q = value;
            obj.childLink.update(obj.q, obj.type);
            obj.tformAT();
        end
        
    end
end