classdef ControllerSISO < handle
    properties
        u_min
        u_max
    end

    methods
        % A constructor for the controller base.
        function cb = ControllerSISO(u_max)
            cb.u_min = -u_max;
            cb.u_max = u_max;
        end

        function u  = execute(obj, p, v, pd, vd)
            % solve the control problem
            u = obj.executeFunction(p, v, pd, vd);
            % saturate the control input
            u = obj.saturate(u, obj.u_min, obj.u_max);
        end
        
        function [result, ratio, limit] = saturate(~, input, lower_bound, upper_bound)
            result = min([upper_bound, max([lower_bound, input])]);
            limit = max([-lower_bound, upper_bound]);
            ratio = abs(result) / limit;
        end
    end

    methods (Abstract)
        % An abstract executeFunction for all controllers. This should take
        % in the generalised coordinate information and produces a control
        % input.
        u = executeFunction(obj, p, v, pd, vd);
    end    
end
