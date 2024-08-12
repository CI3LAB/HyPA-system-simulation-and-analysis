classdef JointSpaceTrajectory < handle
    properties (SetAccess = private)
        timeVector
        q
        q_dot
        q_ddot
        totalTime 
        timeStep        % The time step for the trajectory

        dim
        robotName
        trajectoryID
    end

    properties (Access = private)
        numSegment
        segment_types
        segment_functions
        setpoints
    end

    methods
        function traj = JointSpaceTrajectory(traj_def)
            traj.robotName = traj_def.robotName;
            traj.trajectoryID = traj_def.trajectoryID;
            traj.timeStep = traj_def.timeStep;
            traj.numSegment = traj_def.numSegment;
            traj.segment_types = traj_def.segment_types;
            traj.segment_functions = traj_def.segment_functions;
            traj.setpoints = traj_def.setpoints;
            traj.totalTime = traj.setpoints{end}.time;
            traj.dim = length(traj.setpoints{1}.q);

            % complete the segment function list
            traj.completeSegmentFunctions();

            % generate reference trajectory in the form of a setpoint sequence
            traj.constructTrajectory();

        end

        function completeSegmentFunctions(obj)

            for i = 1:obj.numSegment
                if strcmp(obj.segment_types{i}, 'linear_interpolation')

                    qs = obj.setpoints{i}.q;
                    ts = obj.setpoints{i}.time;
                    qe = obj.setpoints{i+1}.q;
                    te = obj.setpoints{i+1}.time;
                    I = eye(obj.dim);
                    mat_param = [I, ts*I;
                                 I, te*I];
                    vec_param = [qs; qe];
                    param = mat_param\vec_param;
                    pow = 0;
                    A0 = param(pow*obj.dim+1:pow*obj.dim+obj.dim);
                    pow = 1;
                    A1 =param(pow*obj.dim+1:pow*obj.dim+obj.dim);
                    obj.segment_functions{i} = @(t) JointSpaceTrajectory.LinearSegment(t, A0, A1);

                elseif strcmp(obj.segment_types{i}, 'cubic_interpolation')
                    
                    qs = obj.setpoints{i}.q;
                    qds = obj.setpoints{i}.q_dot;
                    ts = obj.setpoints{i}.time;
                    qe = obj.setpoints{i+1}.q;
                    qde = obj.setpoints{i+1}.q_dot;
                    te = obj.setpoints{i+1}.time;
                    I = eye(obj.dim);
                    Z = zeros(obj.dim);
                    mat_param = [I, ts*I,   ts^2*I, ts^3*I;
                                 Z, I,      2*ts*I, 3*ts^2*I;        
                                 I, te*I,   te^2*I, te^3*I;
                                 Z, I,      2*te*I, 3*te^2*I];
                    vec_param = [qs; qds; qe; qde];
                    param = mat_param\vec_param;
                    pow = 0;
                    A0 = param(pow*obj.dim+1:pow*obj.dim+obj.dim);
                    pow = 1;
                    A1 = param(pow*obj.dim+1:pow*obj.dim+obj.dim);
                    pow = 2;
                    A2 = param(pow*obj.dim+1:pow*obj.dim+obj.dim);
                    pow = 3;
                    A3 = param(pow*obj.dim+1:pow*obj.dim+obj.dim);
                    obj.segment_functions{i} = @(t) JointSpaceTrajectory.CubicSegment(t, A0, A1, A2, A3);

                elseif strcmp(obj.segment_types{i}, 'quintic_interpolation')
                    
                    qs = obj.setpoints{i}.q;
                    qds = obj.setpoints{i}.q_dot;
                    qdds = obj.setpoints{i}.q_ddot;
                    ts = obj.setpoints{i}.time;
                    qe = obj.setpoints{i+1}.q;
                    qde = obj.setpoints{i+1}.q_dot;
                    qdde = obj.setpoints{i+1}.q_ddot;
                    te = obj.setpoints{i+1}.time;
                    I = eye(obj.dim);
                    Z = zeros(obj.dim);
                    mat_param = [I, ts*I,   ts^2*I, ts^3*I,     ts^4*I,     ts^5*I;
                                 Z, I,      2*ts*I, 3*ts^2*I,   4*ts^3*I,   5*ts^4*I;
                                 Z, Z,      2*I,    6*ts*I,     12*ts^2*I,  20*ts^3*I;        
                                 I, te*I,   te^2*I, te^3*I,     te^4*I,     te^5*I;
                                 Z, I,      2*te*I, 3*te^2*I,   4*te^3*I,   5*te^4*I;
                                 Z, Z,      2*I,    6*te*I,     12*te^2*I,  20*te^3*I];
                    vec_param = [qs; qds; qdds; qe; qde; qdde];
                    param = mat_param\vec_param;
                    pow = 0;
                    A0 = param(pow*obj.dim+1:pow*obj.dim+obj.dim);
                    pow = 1;
                    A1 = param(pow*obj.dim+1:pow*obj.dim+obj.dim);
                    pow = 2;
                    A2 = param(pow*obj.dim+1:pow*obj.dim+obj.dim);
                    pow = 3;
                    A3 = param(pow*obj.dim+1:pow*obj.dim+obj.dim);
                    pow = 4;
                    A4 = param(pow*obj.dim+1:pow*obj.dim+obj.dim);
                    pow = 5;
                    A5 = param(pow*obj.dim+1:pow*obj.dim+obj.dim);
                    obj.segment_functions{i} = @(t) JointSpaceTrajectory.QuinticSegment(t, A0, A1, A2, A3, A4, A5);

                end
            end

        end

        function constructTrajectory(obj)
            obj.timeVector = (0:obj.timeStep:obj.totalTime)';
            numSteps = length(obj.timeVector);
            obj.q = cell(1, numSteps);
            obj.q_dot = cell(1, numSteps);
            obj.q_ddot = cell(1, numSteps);

            current_segment = 1;
            for i = 1:numSteps
                t = obj.timeVector(i);
                if t > obj.setpoints{current_segment+1}.time
                    current_segment = current_segment + 1;
                end
                [current_q, current_q_dot, current_q_ddot] = obj.segment_functions{current_segment}(t);
                obj.q{i} = current_q;
                obj.q_dot{i} = current_q_dot;
                obj.q_ddot{i} = current_q_ddot;
            end
        end
    end

    methods (Static)

        function [q, q_dot, q_ddot] = LinearSegment(t, A0, A1)
            q = A0 + A1*t;
            q_dot = A1;
            q_ddot = zeros(size(q));
        end

        function [q, q_dot, q_ddot] = CubicSegment(t, A0, A1, A2, A3)
            q = A0 + A1*t + A2*t^2 + A3*t^3;
            q_dot = A1 + 2*A2*t + 3*A3*t^2;
            q_ddot = 2*A2 + 2*3*A3*t;
        end

        function [q, q_dot, q_ddot] = QuinticSegment(t, A0, A1, A2, A3, A4, A5)
            q = A0 + A1*t + A2*t^2 + A3*t^3 + A4*t^4 + A5*t^5;
            q_dot = A1 + 2*A2*t + 3*A3*t^2 + 4*A4*t^3 + 5*A5*t^4;
            q_ddot = 2*A2 + 6*A3*t + 12*A4*t^2 + 20*A5*t^3;
        end
    end
end