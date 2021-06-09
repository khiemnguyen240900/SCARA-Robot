classdef C_Link < dlnode
    properties (Access = private)
        index;
        a; alpha; d; theta;
    end
    
    methods (Access = public)
        function obj = C_Link(index,DH_params,previousLink)
            if (nargin == 2)||(nargin == 3)
                obj.index = index;
                obj.a = DH_params(1);
                obj.alpha = DH_params(2);
                obj.d = DH_params(3);
                obj.theta = DH_params(4);
                if nargin == 3
                    obj.insertAfter(previousLink);
                end
            else
                error('Wrong number of input arguments.')
            end                
        end
        
        function result = get(obj,params)
            switch params
                case 'index'
                    result = obj.index;
                case 'a'
                    result = obj.a;
                case 'alpha'
                    result = obj.alpha;
                case 'd'
                    result = obj.d;
                case 'theta'
                    result = obj.theta;
                case 'DH_params'
                    result = [obj.a, obj.alpha, obj.d, obj.theta];
                otherwise
                    error('Wrong number of input arguments.');   
            end
        end
        
        function update(obj,value,jointType)
            switch jointType
                case 'fixed'
                case 'revolute'
                    obj.theta = value;
                case 'prismatic'
                    obj.d = value;
            end
        end
    end
end