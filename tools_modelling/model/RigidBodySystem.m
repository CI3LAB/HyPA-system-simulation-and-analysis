% a generic system model class that supports both cable actuation and propeller actuation (add joint actuation later if this version works out)
classdef RigidBodySystem < handle

    properties (SetAccess = protected)

        robotName

        % body definition
        CoM     % each link's center of mass coordinate in local frame, a 2D array with the second dimension being the link index
        m       % each link's mass, a row vector
        I_CoM   % each link's inertia tensors w.r.t. the corresponding CoM, a 3-by-3-by-numLinks array
        PL      % each joint's coordinate in the parent link frame, a 2D array with the second dimension being the link index
        VL      % the set of vertices of each link representing their geometries
        parent_link
                % stores the index of parent link for each link
        joint_type_indices
                % a 1D array with joint type internal indices
        link_name
                % a 1D cell array with link name strings
                
        % active joint definition
        joint_active
                % a 1D array showing the joint is active or passive, 1 is active and 0 is passive
        active_dofs
                % a 1D array collecting the indices of all the active dofs


        % cable definition
        cables
                % the data entry indices of the attachments each cable has in the data files, a 1D cell array
        cable_data_attachments
                % each cable's attachment locations in link frame, a 2D array with the second dimension being the attachment index
        cable_data_attachment_link_indices
                % each cable attachment's attached frame, a row vector
        
        
        % propeller definition
        propellers
                % the data entry indices of the propellers in the data files, a 1D array
        propeller_data_link_indices
                % the indices of locations propeller's attached frame
        propeller_data_directions
                % each propeller's pointing direction in link frame, a 2D array with the second dimension being the propeller index
        propeller_data_c
                % each propeller's momentum-lift coefficient, a 1D array
        propeller_data_locations
                % each propeller's install position in link frame, a 2D array with the second dimension being the propeller index
        
                
        % hyper parameter definition
        dimPosition
        dimVelocity
        dimPositionEachLink
        dimVelocityEachLink
        numLinks
        numCableAttachments
        numCables
                % depends on the length of 'cables'
        numPropellers
                % depends on the length of 'propellers'
        numActuators
                % sum of the above two
        numActiveDofs
                % number of active dofs, depends on the length of active_dofs
        


        % - system state
        x_pos       % the system configuration, a vector
        x_pos_dot   % the system configuration derivative, a vector
        x_vel       % the system velocity, a vector
        x_vel_dot   % the system acceleration, a vector
                    % NOTE that unless quaternions are involved in x_pos, x_pos_dot and x_vel are the same
        wext        % the external wrench the systems is subjected to
        
        % - dynamics related properties
        M           % EoM term
        C           % EoM term
        G           % EoM term
        L           % EoM term (the cable Jacobian)
        T           % EoM term (propeller actuation Jacobian)
        W           % EoM term (active joint actuation Jacobian)
        A           % = [W, T]. This naming aligns with CASPR while here the matrix A involves both active joint mapping W and propeller mapping T
        AM          % the complete actuation mapping: AM = [-L', W, T]

        % - control input
        u           % the control input composed of thruster forces
        
    end
    
    properties
        % active joint actuation property
        activeJointForcesMin
        activeJointForcesMax
        % cable actuation property
        cableForcesMin
        cableForcesMax
        % thruster actuation property
        thrusterForcesMin
        thrusterForcesMax
        
        % combine above vectors in the order of: cable -> active joint -> thruster
        % the order corresponds to the definition of actuation mapping matrix AM
        actuationForcesMin
        actuationForcesMax

        % variables that exist only to mimic CASPR model
        actuationForces
        cableModel
        bodyModel
    end

    properties (Dependent)
        
        % these variables exist to align with CASPR model
        numDofs
        numDofVars
        numActuatorsActive
        q_deriv
        q_ddot_dynamics
        
        q
        q_dot

    end

    properties (SetAccess = protected)
        % constants
        g       % gravitational constant
        % configuration independent
        I_s     % the spatial inertia tensor w.r.t. the local frame origin, a 6-by-6-by-numLinks array

        % intermediate variables
        dof_interval_links      % 2-by-numLinks array, representing the index interval of dofs for the specific link
        S_links                 % 6-by-dimVelocity array
        S_dot_links             % 6-by-dimVelocity array
        quat_links              % 4-by-numLinks array
        XM_links                % 6-by-6-by-numLinks array
        XF_links                % 6-by-6-by-numLinks array
        vel_links               % 6-by-numLinks array, each link's velocity in local frame, angular velocity comes first
        OP_links                % 3-by-numLinks array, each links frame origin coordinate in the local frame
        LinkJacobianLibrary     % 6-by-dimVelocity-by-numLinks(-) array containing all the relavent link Jocabians that will be used to compute the actuation mapping matrix
                                %   no need to initialize it, it will be automatically initialized when updating the actuation mapping
    end

    methods
        % A constructor for a computed torque controller.
        function mdl = RigidBodySystem(struct_model_def)
            
            mdl.robotName                           =   struct_model_def.robotName;

            % extract info from model definition struct

            % body definition
            mdl.CoM                                 =   struct_model_def.CoM;
            mdl.m                                   =   struct_model_def.m;
            mdl.I_CoM                               =   struct_model_def.I_CoM;
            mdl.PL                                  =   struct_model_def.PL;
            mdl.VL                                  =   struct_model_def.VL;
            mdl.parent_link                         =   struct_model_def.parent_link;
            mdl.joint_active                        =   struct_model_def.joint_active;
            if isfield(struct_model_def, 'link_name')
                mdl.link_name                           =   struct_model_def.link_name;
            else
                mdl.link_name                           =   cell(size(mdl.m));
            end
            
            % cable and propeller definitions
            mdl.cables                              =   struct_model_def.cables;
            mdl.cable_data_attachments              =   struct_model_def.cable_data_attachments;
            mdl.cable_data_attachment_link_indices  =   struct_model_def.cable_data_attachment_link_indices;
            mdl.propellers                          =   struct_model_def.propellers;
            mdl.propeller_data_link_indices         =   struct_model_def.propeller_data_link_indices;
            mdl.propeller_data_directions           =   struct_model_def.propeller_data_directions;
            mdl.propeller_data_c                    =   struct_model_def.propeller_data_c;
            mdl.propeller_data_locations            =   struct_model_def.propeller_data_locations;
        
            % actuation properties
            mdl.activeJointForcesMin                =   struct_model_def.activeJointForcesMin;
            mdl.activeJointForcesMax                =   struct_model_def.activeJointForcesMax;
            mdl.cableForcesMin                      =   struct_model_def.cableForcesMin;
            mdl.cableForcesMax                      =   struct_model_def.cableForcesMax;
            mdl.thrusterForcesMin                   =   struct_model_def.thrusterForcesMin;
            mdl.thrusterForcesMax                   =   struct_model_def.thrusterForcesMax;
            mdl.actuationForcesMin                  =   [mdl.cableForcesMin; mdl.activeJointForcesMin; mdl.thrusterForcesMin];
            mdl.actuationForcesMax                  =   [mdl.cableForcesMax; mdl.activeJointForcesMax; mdl.thrusterForcesMax];
        
            % hyperparameters
            mdl.numLinks                    =   length(mdl.m);
            mdl.numCables                   =   length(mdl.cables);
            mdl.numPropellers               =   length(mdl.propellers);
            mdl.dimPositionEachLink         =   zeros(mdl.numLinks, 1);
            mdl.dimVelocityEachLink         =   zeros(mdl.numLinks, 1);
            mdl.joint_type_indices          =   zeros(mdl.numLinks, 1);
            mdl.active_dofs                 =   [];
            dof_counter                     =   0;
            for i = 1:mdl.numLinks
                mdl.joint_type_indices(i)   =   joint_type_interpretation(struct_model_def.joint_type{i});
                [pos_dof, vel_dof]          =   joint_dofs(mdl.joint_type_indices(i));
                mdl.dimPositionEachLink(i)  =   pos_dof;
                mdl.dimVelocityEachLink(i)  =   vel_dof;
                if mdl.joint_active(i)
                    mdl.active_dofs = [mdl.active_dofs; (dof_counter+1:dof_counter+vel_dof)'];
                end
                dof_counter = dof_counter + vel_dof;
            end
            mdl.dimPosition                 =   sum(mdl.dimPositionEachLink);
            mdl.dimVelocity                 =   sum(mdl.dimVelocityEachLink);
            mdl.numActiveDofs               =   length(mdl.active_dofs);
            mdl.numActuators                =   mdl.numCables + mdl.numPropellers + mdl.numActiveDofs;

            % derive active joint actuation Jacobian
            mdl.W = zeros(mdl.dimVelocity, mdl.numActiveDofs);
            for i = 1:mdl.numActiveDofs
                mdl.W(mdl.active_dofs(i), i) = 1;
            end


            % define private properties
            mdl.g = 9.81;
            mdl.I_s = zeros(6, 6, mdl.numLinks);
            for i = 1:mdl.numLinks
                mdl.I_s(:,:,i) = spatialInertiaTensor(mdl.I_CoM{i}, mdl.CoM(:,i), mdl.m(i));
            end

            % initialize the intermediate variables
            mdl.dof_interval_links...
                                =   zeros(2, mdl.numLinks);
            mdl.S_links         =   zeros(6, mdl.dimVelocity);
            mdl.S_dot_links     =   zeros(6, mdl.dimVelocity);
            mdl.quat_links      =   zeros(4, mdl.numLinks);
            mdl.XM_links        =   zeros(6, 6, mdl.numLinks);
            mdl.XF_links        =   zeros(6, 6, mdl.numLinks);
            mdl.vel_links       =   zeros(6, mdl.numLinks);
            mdl.OP_links        =   zeros(3, mdl.numLinks);

            % initialize the system state record
            mdl.x_pos           =   zeros(mdl.dimPosition, 1);
            mdl.x_vel           =   zeros(mdl.dimVelocity, 1);
            mdl.x_pos_dot       =   zeros(mdl.dimPosition, 1);
            mdl.x_vel_dot       =   zeros(mdl.dimVelocity, 1);
            mdl.wext            =   zeros(mdl.dimVelocity, 1);

        end
        
        function attachment_set = extractAttachmentSetForGivenCableSet(obj, cable_idx_set, target_link)
            if ischar(target_link)
                target_link_idx = obj.getLinkIndex(target_link);
            else
                target_link_idx = target_link;
            end
            num_cables = length(cable_idx_set);
            attachment_set = cell(1,num_cables);
            for i = 1:num_cables
                cable_idx = cable_idx_set(i);
                cable_attachment_set = obj.cables{cable_idx};
                num_cable_attachment = length(cable_attachment_set);
                relevant_attachment_idx_set = [];
                for j = 1:num_cable_attachment
                    cable_attachment_idx = cable_attachment_set(j);
                    cable_attachment_link_idx = obj.cable_data_attachment_link_indices(cable_attachment_idx);
                    if cable_attachment_link_idx == target_link_idx
                        relevant_attachment_idx_set = [relevant_attachment_idx_set, cable_attachment_idx];
                    end
                end
                attachment_set{i} = obj.cable_data_attachments(:, relevant_attachment_idx_set);
            end
        end
        
        function idx = getLinkIndex(obj, name_string)
            idx = -1;
            for i = 1:length(obj.link_name)
                if strcmp(obj.link_name{i}, name_string)
                    idx = i;
                end
            end
        end

        function updateSystemState(obj, x_pos, x_vel)

            obj.x_pos = x_pos;
            obj.x_vel = x_vel;

            x_pos_count =   0;
            x_vel_count =   0;

            OP_0        =   [0; 0; 0];
            quat_0      =   [1; 0; 0; 0];
            v_0      =   [0; 0; 0; 0; 0; 0];

            for i = 1:obj.numLinks
                % update the rotation matrix from frame 0 to frame i using each link's joint information
                
                % extract the information of joint i
                dimPosition_i           =   obj.dimPositionEachLink(i);
                dimVelocity_i           =   obj.dimVelocityEachLink(i);
                interval_pos            =   x_pos_count+1 : x_pos_count+dimPosition_i;
                interval_vel            =   x_vel_count+1 : x_vel_count+dimVelocity_i;
                x_pos_i                 =   x_pos(interval_pos);
                x_vel_i                 =   x_vel(interval_vel);
                obj.dof_interval_links(1,i)     =   x_vel_count+1;
                obj.dof_interval_links(2,i)     =   x_vel_count+dimVelocity_i;
                x_pos_count             =   x_pos_count + dimPosition_i;
                x_vel_count             =   x_vel_count + dimVelocity_i;
                
                % relative motion of link i (w.r.t. the parent link)
                joint_type_i            =   obj.joint_type_indices(i);
                PL_i                    =   obj.PL(:,i) + joint_translation(joint_type_i, x_pos_i);
                [S_i, S_deriv_i]        =   joint_S(joint_type_i, x_pos_i, x_vel_i);
                quat_i                  =   joint_quat(joint_type_i, x_pos_i);
                R_i                     =   QuaternionToRotationMatrix(quat_i);

                % parent link index
                p_i                     =   obj.parent_link(i);

                % transformation matrix from p_i to i (motion type) and
                % from i to p_i (force type)
                X_M_i                   =   rotMat(R_i)*xltMMat(PL_i);
                X_F_i                   =   xltFMat(PL_i)*rotMat(R_i');
                
                % accumulated properties of link i
                if p_i ~= 0
                    quat                    =   QuaternionMultiplication(obj.quat_links(:,p_i), quat_i);
                    % OP_i                    =   QuaternionVectorRotation(quat_i, obj.OP_links(:,p_i) + PL_i); % (this is actually slower...) propagate the link frame origin coordinate
                    OP_i                    =   R_i*(obj.OP_links(:,p_i) + PL_i); % propagate the link frame origin coordinate
                    v_i                     =   X_M_i*obj.vel_links(:,p_i) + S_i*x_vel_i;
                else
                    quat                    =   QuaternionMultiplication(quat_0, quat_i);
                    % OP_i                    =   QuaternionVectorRotation(quat_i, OP_0 + PL_i); % (this is actually slower...) propagate the link frame origin coordinate
                    OP_i                    =   R_i*(OP_0 + PL_i); % propagate the link frame origin coordinate
                    v_i                     =   X_M_i*v_0 + S_i*x_vel_i;
                end

                S_dot_i     =   vecX6DM(v_i)*S_i + S_deriv_i;


                % take records
                obj.S_links(:,interval_vel)    	=   S_i;
                obj.S_dot_links(:,interval_vel)	=   S_dot_i;
                obj.quat_links(:,i)             =   quat;
                obj.XM_links(:,:,i)             =   X_M_i;
                obj.XF_links(:,:,i)             =   X_F_i;
                obj.vel_links(:,i)              =   v_i;
                obj.OP_links(:,i)               =   OP_i;

                obj.x_pos_dot(interval_pos) = joint_qderiv(joint_type_i, x_pos_i, x_vel_i);
            end

        end

        function link_set = findSubtree(obj, linkIndex)
            link_set = [];
            link_set_current_level = linkIndex;
            while ~isempty(link_set_current_level)
                link_set = [link_set, link_set_current_level];
                tmp = link_set_current_level;
                link_set_current_level = [];
                for i = tmp
                    link_set_current_level = [link_set_current_level, find(obj.parent_link==i)];
                end
            end
        end

        function link_set = findRootPath(obj, linkIndex)
            link_set = [];
            current_link = linkIndex;
            while current_link ~= 0
                link_set = [current_link, link_set];
                current_link = obj.parent_link(current_link);
            end
        end

        % the function returns the branch of a path (i.e. with the proximal extreme removed)
        % between two given links
        function link_set = findBranchOfIntervalPath(obj, startLinkIndex, endLinkIndex)
            if startLinkIndex == endLinkIndex
                link_set = [];
            elseif startLinkIndex == 0
                endLinkRootPath = obj.findRootPath(endLinkIndex);
                link_set = endLinkRootPath;
            elseif endLinkIndex == 0
                startLinkRootPath = obj.findRootPath(startLinkIndex);
                link_set = startLinkRootPath(end:-1:1);
            else
                startLinkRootPath = obj.findRootPath(startLinkIndex);
                endLinkRootPath = obj.findRootPath(endLinkIndex);
                lenStartLinkRootPath = length(startLinkRootPath);
                lenEndLinkRootPath = length(endLinkRootPath);
                intersectionFound = false;
                for i = 1:lenStartLinkRootPath
                    startLinkPathElement = startLinkRootPath(1-i+lenStartLinkRootPath);
                    for j = 1:lenEndLinkRootPath
                        endLinkPathElement = endLinkRootPath(1-j+lenEndLinkRootPath);
                        if startLinkPathElement == endLinkPathElement
                            intersectionFound = true;
                        end
                        if intersectionFound
                            break;
                        end
                    end
                    if intersectionFound
                        break;
                    end
                end
                startLinkToIntersectionLinkPath = startLinkRootPath(lenStartLinkRootPath:-1:lenStartLinkRootPath+1-i);
                intersectionChildLinkToEndLinkPath = endLinkRootPath(lenEndLinkRootPath+2-j:1:lenEndLinkRootPath);
                link_set = [startLinkToIntersectionLinkPath, intersectionChildLinkToEndLinkPath];
                link_set = obj.removeProximalEndFromLinkChain(link_set); % remove the proximal end if base is not in the link set
            end
        end
        
        function link_chain = removeProximalEndFromLinkChain(obj, link_chain)
            proximal_end_found = false;
            proximal_end_idx = -1;
            for i = 1:length(link_chain)
                link_i = link_chain(i);
                link_i_parent = obj.parent_link(link_i);
                num_of_hit = sum(link_chain == link_i_parent);
                if num_of_hit == 0
                    proximal_end_idx = i;
                    proximal_end_found = true;
                elseif num_of_hit > 1
                    error('the given link chain has loop');
                end
                if proximal_end_found == true
                    break;
                end
            end
            if proximal_end_found
                link_chain(proximal_end_idx) = [];
            else
                error('faild to find the proximal end of the given chain');
                link_chain = [];
            end
        end
        
        % this function finds dofs that are within the link set. i.e.
        % assume that the most proximal link is fixed then get the
        % available dofs
        function dof_set = findDofSet(obj, link_chain)
            link_chain = sort(link_chain);
            dof_set = [];
            for i = 1:length(link_chain)
                link_idx = link_chain(i);
                dof_set = [dof_set, obj.dof_interval_links(1,link_idx):obj.dof_interval_links(2,link_idx)];
            end
        end
        
        % CoM coordinate in the world frame
        function com = centerOfMassCoordinate(obj, link_set, link_to_represent_in)
            if nargin < 2
                link_set = 1:obj.numLinks;
                link_to_represent_in = 0.0;
            elseif nargin < 3
                link_to_represent_in = 0.0;
            end
            com = zeros(3,1);
            mass_sum = 0.0;
            for i = 1:length(link_set)
                link_idx_i = link_set(i);
                link_idx_i = min([obj.numLinks, max([1, link_idx_i])]);
                mass_sum = mass_sum + obj.m(link_idx_i);
                com = com + obj.m(link_idx_i)*obj.pointCoordinate(link_idx_i, obj.CoM(:,link_idx_i), link_to_represent_in);
            end
            com = com / mass_sum;
        end
        % CoM jacobian in the world frame
        function [comJac, comJac_dot] = centerOfMassJacobian(obj, link_set)
            if nargin == 1
                link_set = 1:obj.numLinks;
            end
            comJac = zeros(3,obj.numDofs);
            comJac_dot = zeros(3,obj.numDofs);
            mass_sum = 0.0;
            for i = 1:length(link_set)
                link_idx_i = link_set(i);
                link_idx_i = min([obj.numLinks, max([1, link_idx_i])]);
                mass_sum = mass_sum + obj.m(link_idx_i);
                [comiJac, comiJac_dot] = obj.pointJacobian(link_idx_i, obj.CoM(:,link_idx_i), 0);
                comJac = comJac + obj.m(link_idx_i)*comiJac;
                comJac_dot = comJac_dot + obj.m(link_idx_i)*comiJac_dot;
            end
            comJac = comJac / mass_sum;
            comJac_dot = comJac_dot / mass_sum;
        end
        
        % function that returns a link's operational space Jacobian and its
        % derivative, the velocity and acceleration are represented in the 
        % local frame with angular velocity/acc comes first.
        % R_lib collects the rotation matrices from ground frame to link
        % frames
        % Further explanations on the computation and use of operational Jacobian dot.
        % Here JL_dot is the apparent derivative of JL (i.e. numerical)
        % Denote the link velocity as v, then v_dot = J_dot*q_dot + J*q_ddot holds for apparent derivative J_dot iff the selected reference frame is stationary (i.e. the ground frame)
        % Obviously it is not the case here since the default reference frame here is the local frame. 
        function [JL, JL_dot] = linkJacobian(obj, linkIndex)
            
            % assumes that the system state is updated, i.e. obj.updateSystemState(x_pos, x_vel) is called

            if linkIndex <= 0
                JL          =   zeros(6, obj.dimVelocity);
                JL_dot      =   zeros(6, obj.dimVelocity);
            else
                if linkIndex > obj.numLinks
                    linkIndex = obj.numLinks;
                end
                % initialize the operational space link Jacobian (converts the joint
                % velocity into the spatial velocity of the link that the operational
                % space attached to)
                JL          =   zeros(6, obj.dimVelocity);
                JL_dot      =   zeros(6, obj.dimVelocity);
                % find the path to the selected link
                link_set = obj.findRootPath(linkIndex);
                for i = link_set
                    interval_dimVel_i = obj.dof_interval_links(1,i):obj.dof_interval_links(2,i);
                    JL              =   obj.XM_links(:,:,i)*JL;
                    JL(:,interval_dimVel_i)... 
                                    =   obj.S_links(:,interval_dimVel_i);
                    JL_dot          =   obj.XM_links(:,:,i)*JL_dot;
                    JL_dot(:,interval_dimVel_i)...
                                    =   obj.S_dot_links(:,interval_dimVel_i);
                end

                % the velocity of the operational space link is v_i, the following is
                % the extracted translational velocity of that link
                vel_selected        =   obj.vel_links(:,linkIndex);
                % vel_selected(1:3)   =   [0; 0; 0];
                % similarly the spatial acceleration of the operational space link is
                % a_i
                % then the classical acceleration can be extracted from the spatial
                % acceleration vector by:
                % a = a_i - vecX6DM(vel_i)*v_i;
                JL_dot = JL_dot - vecX6DM(vel_selected)*JL;
            end

        end
        
        % function that gives out the Jacobians and Jacobian derivatives of
        % points on a certain link
        % the input point should be represented in the local frame of the
        % selected link
        % Further explanations on the computation and use of operational Jacobian dot.
        % Here Jp_dot is the apparent derivative of Jp (i.e. numerical)
        % Denote the point velocity as v, then v_dot = J_dot*q_dot + J*q_ddot holds for apparent derivative J_dot iff the selected reference frame is stationary (i.e. the ground frame)
        % For the Jacobian/Jacobian derivative to be useful in practice the reference frame (third input) shall be selected as base frame (which obviously has zero velocity).
        function [Jp, Jp_dot] = pointJacobian(obj, opLinkIndex, pointCoordinate, frameLinkIndex)

            % safe guard the given point link index.
            if opLinkIndex > obj.numLinks
                opLinkIndex = obj.numLinks;
                quat_op_link = obj.quat_links(:,opLinkIndex);
            elseif opLinkIndex < 1
                opLinkIndex = 0;
                quat_op_link = [1; 0; 0; 0];
            else
                quat_op_link = obj.quat_links(:,opLinkIndex);
            end
            if opLinkIndex == 0
                opLinkOmega = zeros(3,1);
            else
                opLinkOmega = obj.vel_links(1:3,opLinkIndex);
            end
            % derive the corresponding link jacobian and its derivative
            [JL, JL_dot] = obj.linkJacobian(opLinkIndex);
            % safe guard the frame link index
            if nargin == 3
                frameLinkIndex = opLinkIndex;
            end
            if frameLinkIndex > obj.numLinks
                frameLinkIndex = obj.numLinks;
                quat_frame_link = obj.quat_links(:,frameLinkIndex);
            elseif frameLinkIndex < 1
                frameLinkIndex = 0;
                quat_frame_link = [1; 0; 0; 0];
            else
                quat_frame_link = obj.quat_links(:,frameLinkIndex);
            end
            if frameLinkIndex == 0
                frameLinkOmega = zeros(3,1);
            else
                frameLinkOmega = obj.vel_links(1:3,frameLinkIndex);
            end

            if frameLinkIndex == opLinkIndex
                J = xltMMat(pointCoordinate)*JL;
                J_dot = xltMMat(pointCoordinate)*JL_dot;
                Jp = J(4:6,:);
                Jp_dot = J_dot(4:6,:);
            else
                delta_quat = QuaternionMultiplication(QuaternionConjugate(quat_op_link), quat_frame_link);
                R = QuaternionToRotationMatrix(delta_quat);
                delta_omega = frameLinkOmega - R*opLinkOmega;
                
                J = rotMat(R)*xltMMat(pointCoordinate)*JL;
                % the derivative of rotation matrix is used here: -(omega_B - omega_A)_in_frame_B \times R_A_to_B = R_A_to_B_dot
                J_dot = rotMat(R)*xltMMat(pointCoordinate)*JL_dot + rotMat(vecX3D(-delta_omega))*J;
                Jp = J(4:6,:);
                Jp_dot = J_dot(4:6,:);
            end
            
        end

        
        
        % function that gives out the Jacobians and Jacobian derivatives of
        % the orientation of a certain link
        % the input point should be represented in the local frame of the
        % selected link
        % the left side of the Jacobian is also represented in the local frame
        % Further explanations on the computation and use of operational Jacobian dot.
        % Here Jo_dot is the apparent derivative of Jo (i.e. numerical)
        % Denote the angular velocity as w, then w_dot = J_dot*q_dot + J*q_ddot holds for apparent derivative J_dot iff the selected reference frame is stationary (i.e. the ground frame)
        % For the Jacobian/Jacobian derivative to be useful in practice the reference frame (third input) shall be selected as base frame (which obviously has zero velocity).
        function [Jo, Jo_dot] = orientationJacobian(obj, opLinkIndex, frameLinkIndex)

            % assumes that the system state is updated, i.e. obj.updateSystemState(x_pos, x_vel) is called

            % safe guard the given point link index.
            if opLinkIndex > obj.numLinks
                opLinkIndex = obj.numLinks;
                quat_op_link = obj.quat_links(:,opLinkIndex);
            elseif opLinkIndex < 1
                opLinkIndex = 0;
                quat_op_link = [1; 0; 0; 0];
            else
                quat_op_link = obj.quat_links(:,opLinkIndex);
            end
            if opLinkIndex == 0
                opLinkOmega = zeros(3,1);
            else
                opLinkOmega = obj.vel_links(1:3,opLinkIndex);
            end
            % derive the corresponding link jacobian and its derivative
            [JL, JL_dot] = obj.linkJacobian(opLinkIndex);
            % safe guard the frame link index
            if nargin == 2
                frameLinkIndex = opLinkIndex;
            end
            if frameLinkIndex > obj.numLinks
                frameLinkIndex = obj.numLinks;
                quat_frame_link = obj.quat_links(:,frameLinkIndex);
            elseif frameLinkIndex < 1
                frameLinkIndex = 0;
                quat_frame_link = [1; 0; 0; 0];
            else
                quat_frame_link = obj.quat_links(:,frameLinkIndex);
            end
            if frameLinkIndex == 0
                frameLinkOmega = zeros(3,1);
            else
                frameLinkOmega = obj.vel_links(1:3,frameLinkIndex);
            end

            if frameLinkIndex == opLinkIndex
                Jo = JL(1:3,:);
                Jo_dot = JL_dot(1:3,:);
            else
                delta_quat = QuaternionMultiplication(QuaternionConjugate(quat_op_link), quat_frame_link);
                R = QuaternionToRotationMatrix(delta_quat);
                delta_omega = frameLinkOmega - R*opLinkOmega;
                
                J = rotMat(R)*JL;
                % the derivative of rotation matrix is used here: -(omega_B - omega_A)_in_frame_B \times R_A_to_B = R_A_to_B_dot
                J_dot = rotMat(R)*JL_dot + rotMat(vecX3D(-delta_omega))*J;
                Jo = J(1:3,:);
                Jo_dot = J_dot(1:3,:);
            end
        end

        % function that calculates cable length for the specific cable
        function len = cableLength(obj, cableIdx)
            cableAttachmentSet = obj.cables{cableIdx};
            numCableAttachmentsOfSelectedCable = length(cableAttachmentSet);
            cableAttachmentCoordSet = zeros(3, numCableAttachmentsOfSelectedCable);
            for i = 1:numCableAttachmentsOfSelectedCable
                attachmentIdx = cableAttachmentSet(i);
                attachmentLinkIdx = obj.cable_data_attachment_link_indices(attachmentIdx);
                attachmentCoord = obj.cable_data_attachments(:, attachmentIdx);
                cableAttachmentCoordSet(:, i) = obj.pointCoordinate(attachmentLinkIdx, attachmentCoord, 0);
            end
            len = 0;
            prevAttachmentCoord = cableAttachmentCoordSet(:, 1);
            for i = 2:numCableAttachmentsOfSelectedCable
                currentAttachmentCoord = cableAttachmentCoordSet(:, i);
                len = len + norm(currentAttachmentCoord - prevAttachmentCoord);
                prevAttachmentCoord = currentAttachmentCoord;
            end
        end
        
        % function that gives out the the coordinate of a certain point on a certain link represented in a certain link's frame
        % the input point should be represented in the local frame of the selected link
        function y = pointCoordinate(obj, pointLinkIndex, pointCoordinate, frameLinkIndex)

            % assumes that the system state is updated, i.e. obj.updateSystemState(x_pos, x_vel) is called

            % get target frame origin coordinate in the base frame and the target frame quaternion
            if frameLinkIndex == 0
                OP_frame_link = [0; 0; 0];
                quat_frame_link = [1; 0; 0; 0];
            else
                OP_frame_link = obj.OP_links(:,frameLinkIndex);
                quat_frame_link = obj.quat_links(:,frameLinkIndex);
            end
            if pointLinkIndex == 0
                OP_point_link = [0; 0; 0];
                quat_point_link = [1; 0; 0; 0];
            else
                OP_point_link = obj.OP_links(:,pointLinkIndex);
                quat_point_link = obj.quat_links(:,pointLinkIndex);
            end

            
            % get the coordinate of the point of interest in the base frame
            
            y = repmat(OP_point_link, 1, size(pointCoordinate, 2)) + pointCoordinate;
            quat_delta = QuaternionMultiplication(QuaternionConjugate(quat_point_link), quat_frame_link);
            % y = QuaternionVectorRotation(quat_delta, y) - OP_frame_link; % this is actually slower...
            y = QuaternionToRotationMatrix(quat_delta)*y - repmat(OP_frame_link, 1, size(pointCoordinate, 2));
            
        end


        % function that gives out the the orientation (quaternion or euler angles) of a certain link
        function quat = linkOrientationQuaternion(obj, linkIndex)

            % assumes that the system state is updated, i.e. obj.updateSystemState(x_pos, x_vel) is called
            
            quat = obj.quat_links(:,linkIndex);

        end
        function [alpha, beta, gamma] = linkOrientationEulerXYZ(obj, linkIndex)

            % assumes that the system state is updated, i.e. obj.updateSystemState(x_pos, x_vel) is called
            
            [alpha, beta, gamma] = QuaternionToEulerXYZ(obj.quat_links(:,linkIndex));

        end

        
        % function that returns actuator velocities in the local
        % frame
        function vel_t = propellerVelocity(obj, propellerIndex)

            if propellerIndex < 1 && propellerIndex > obj.numPropellers
                error('The enquired propeller (indexed as %i) does not exist in the model.', propellerIndex);
            else
                propeller_frame_index = obj.propeller_data_link_indices(obj.propellers(propellerIndex));
                propeller_position = obj.propeller_data_locations(:, obj.propellers(propellerIndex));
                [Jp, ~] = obj.pointJacobian(propeller_frame_index, propeller_position, propeller_frame_index);
                vel_index_ub = sum(obj.dimVelocityEachLink(1:propeller_frame_index));
                vel_index_lb = 1;
                vel_t = Jp(:, vel_index_lb:vel_index_ub)*obj.x_vel(vel_index_lb:vel_index_ub);
            end
        end


        function vel_p = pointVelocity(obj, pointLinkIndex, pointCoordinate, frameLinkIndex)
            [Jp, ~] = obj.pointJacobian(pointLinkIndex, pointCoordinate, frameLinkIndex);
            vel_p = Jp*obj.x_vel;
        end

        
        % rigid body dynamics functions
        % function M
        function M = updateM(obj)
            % two versions here, use cell array or use regular array, see which one is faster

            M_cell      =   cell(obj.numLinks, obj.numLinks);
            f_comp      =   cell(obj.numLinks, obj.numLinks);
            I_s_comp    =   obj.I_s;

            % % initialize M_cell into a zero matrix subdivision
            % for i = 1:obj.numLinks
            %     dof_i = obj.dimVelocityEachLink(i);
            %     for j = i+1:obj.numLinks
            %         dof_j = obj.dimVelocityEachLink(j);
            %         M_cell{i,j} = zeros(dof_i,dof_j);
            %     end
            % end
            % for j = 1:obj.numLinks
            %     i = obj.numLinks + 1 - j;
            %     interval_dimVel_i = obj.dof_interval_links(1,i):obj.dof_interval_links(2,i);
            %     f_comp{i,i} = I_s_comp(:,:,i)*obj.S_links(:,interval_dimVel_i);
            %     link_set = obj.findSubtree(i);
            %     for k = link_set
            %         M_cell{i,k} = obj.S_links(:,interval_dimVel_i)'*f_comp{k,i};
            %         M_cell{k,i} = M_cell{i,k}';
            %         % M_cell{k,i} = obj.S_links(:,interval_dimVel_i)'*f_comp{k,i};
            %         % M_cell{i,k} = M_cell{k,i}';
            %     end
            %     p_i = obj.parent_link(i);
            %     if p_i ~= 0
            %         XF_b = obj.XF_links(:,:,i);
            %         XM_f = obj.XM_links(:,:,i);
            %         I_s_comp(:,:,p_i) = I_s_comp(:,:,p_i) + XF_b*I_s_comp(:,:,i)*XM_f;
            %         link_set = obj.findSubtree(i);
            %         for k = link_set
            %             f_comp{k,p_i} = XF_b*f_comp{k,i};
            %         end
            %     end
            % end
            

            % initialize M_cell into a zero matrix subdivision
            for i = 1:obj.numLinks
                dof_i = obj.dimVelocityEachLink(i);
                for j = i+1:obj.numLinks
                    dof_j = obj.dimVelocityEachLink(j);
                    M_cell{i,j} = zeros(dof_i,dof_j);
                    M_cell{j,i} = M_cell{i,j}';
                end
            end
            for j = 1:obj.numLinks
                i = obj.numLinks + 1 - j;
                interval_dimVel_i = obj.dof_interval_links(1,i):obj.dof_interval_links(2,i);
                S_i = obj.S_links(:,interval_dimVel_i);
                f_comp{i,i} = I_s_comp(:,:,i)*S_i;
                link_set = obj.findSubtree(i);
                for k = link_set
                    M_cell{i,k} = S_i'*f_comp{k,i};
                    M_cell{k,i} = M_cell{i,k}';
                end
                p_i = obj.parent_link(i);
                if p_i ~= 0
                    XF_b = obj.XF_links(:,:,i);
                    I_s_comp(:,:,p_i) = I_s_comp(:,:,p_i) + XF_b*I_s_comp(:,:,i)*obj.XM_links(:,:,i);
                    link_set = obj.findSubtree(i);
                    for k = link_set
                        f_comp{k,p_i} = XF_b*f_comp{k,i};
                    end
                end
            end
            
            obj.M   =   cell2mat(M_cell);
            M = obj.M;
            % rM = rank(M)


            % this doesn't work, stick with cell array approach for now
            % obj.M       =   zeros(obj.dimVelocity, obj.dimVelocity);
            % f_comp      =   zeros(obj.numLinks*6, obj.dimVelocity);
            % I_s_comp    =   obj.I_s;
            % for j = 1:obj.numLinks
            %     i = obj.numLinks + 1 - j;
            %     interval_dimVel_i = obj.dof_interval_links(1,i):obj.dof_interval_links(2,i);
            %     tmp = 6*(i-1);
            %     interval_6_i = tmp+1:tmp+6;

            %     f_comp(interval_6_i, interval_dimVel_i) = I_s_comp(:,:,i)*obj.S_links(:,interval_dimVel_i);

            %     for k = i:obj.numLinks
            %         interval_dimVel_k = obj.dof_interval_links(1,k):obj.dof_interval_links(2,k);
            %         tmp = 6*(k-1);
            %         interval_6_k = tmp+1:tmp+6;
                    
            %         obj.M(interval_dimVel_i,interval_dimVel_k) = obj.S_links(:,interval_dimVel_i)'*f_comp(interval_6_k,interval_dimVel_i);
            %         obj.M(interval_dimVel_k,interval_dimVel_i) = obj.M(interval_dimVel_i,interval_dimVel_k)';
            %         % obj.M(interval_dimVel_k,interval_dimVel_i) = obj.S_links(:,interval_dimVel_i)'*f_comp(interval_6_k,interval_dimVel_i);
            %         % obj.M(interval_dimVel_i,interval_dimVel_k) = obj.M(interval_dimVel_k,interval_dimVel_i)';
            %     end
            %     p_i = i - 1;
            %     if p_i ~= 0
            %         interval_dimVel_p_i = obj.dof_interval_links(1,p_i):obj.dof_interval_links(2,p_i);
            %         interval_dimVel_i = obj.dof_interval_links(1,i):obj.dof_interval_links(2,i);

            %         XF_b = obj.XF_links(:,:,i);
            %         XM_f = obj.XM_links(:,:,i);
            %         I_s_comp(:,:,p_i) = I_s_comp(:,:,p_i) + XF_b*I_s_comp(:,:,i)*XM_f;
            %         for k = i:obj.numLinks
            %             tmp = 6*(k-1);
            %             interval_6_k = tmp+1:tmp+6;
            %             f_comp(interval_6_k,interval_dimVel_p_i) = XF_b*f_comp(interval_6_k,interval_dimVel_i);
            %         end
            %     end
            % end
            % M   =   obj.M;
            % % rM = rank(M)
        end
        % function C
        function C = updateC(obj)
            % two versions here, use cell array or use regular array, see which one is faster
            
            % f_lib = cell(obj.numLinks, 1);
            % acc_lib = cell(obj.numLinks, 1);
            % wrench = cell(obj.numLinks, 1);
            
            % acc_0 = zeros(6,1);
            
            % % for i = 1:obj.numLinks
            % %     interval_dimVel_i = obj.dof_interval_links(1,i):obj.dof_interval_links(2,i);
            % %     vq              =   obj.x_vel(interval_dimVel_i);
            % %     vel             =   obj.vel_links(:,i);
            % %     S_dot           =   obj.S_dot_links(:,interval_dimVel_i);
            % %     XM              =   obj.XM_links(:,:,i);
            % %     i_p             =   obj.parent_link(i);
            % %     if i_p ~= 0
            % %         acc_lib{i}  =   XM*acc_lib{i_p} + S_dot*vq;
            % %     else
            % %         acc_lib{i}  =   XM*acc_0 + S_dot*vq;
            % %     end
            % %     f_lib{i}        =   obj.I_s(:,:,i)*acc_lib{i} + vecX6DF(vel)*obj.I_s(:,:,i)*vel;
            % % end
            
            % % for j = 1:obj.numLinks
            % %     i           =   obj.numLinks + 1 - j;
            % %     interval_dimVel_i = obj.dof_interval_links(1,i):obj.dof_interval_links(2,i);
            % %     wrench{i}   =   obj.S_links(:,interval_dimVel_i)'*f_lib{i};
            % %     i_p = obj.parent_link(i);
            % %     if i_p ~= 0
            % %         XF          =   obj.XF_links(:,:,i);
            % %         f_lib{i_p}  =   f_lib{i_p} + XF*f_lib{i};
            % %     end
            % % end
            
            % for i = 1:obj.numLinks
            %     interval_dimVel_i = obj.dof_interval_links(1,i):obj.dof_interval_links(2,i);
            %     vel             =   obj.vel_links(:,i);
            %     i_p             =   obj.parent_link(i);
            %     if i_p ~= 0
            %         acc_lib{i}  =   obj.XM_links(:,:,i)*acc_lib{i_p} + obj.S_dot_links(:,interval_dimVel_i)*obj.x_vel(interval_dimVel_i);
            %     else
            %         acc_lib{i}  =   obj.XM_links(:,:,i)*acc_0 + obj.S_dot_links(:,interval_dimVel_i)*obj.x_vel(interval_dimVel_i);
            %     end
            %     f_lib{i}        =   obj.I_s(:,:,i)*acc_lib{i} + vecX6DF(vel)*obj.I_s(:,:,i)*vel;
            % end
            
            % for j = 1:obj.numLinks
            %     i           =   obj.numLinks + 1 - j;
            %     interval_dimVel_i = obj.dof_interval_links(1,i):obj.dof_interval_links(2,i);
            %     wrench{i}   =   obj.S_links(:,interval_dimVel_i)'*f_lib{i};
            %     i_p = obj.parent_link(i);
            %     if i_p ~= 0
            %         f_lib{i_p}  =   f_lib{i_p} + obj.XF_links(:,:,i)*f_lib{i};
            %     end
            % end
            
            % obj.C = cell2mat(wrench);
            % C = obj.C;

            
            
            f_lib = zeros(6, obj.numLinks);
            acc_lib = zeros(6, obj.numLinks);
            wrench = zeros(obj.dimVelocity, 1);
            
            acc_0 = [0; 0; 0; 0; 0; 0];
            
            % for i = 1:obj.numLinks
            %     interval_dimVel_i = obj.dof_interval_links(1,i):obj.dof_interval_links(2,i);
            %     vq              =   obj.x_vel(interval_dimVel_i);
            %     vel             =   obj.vel_links(:,i);
            %     S_dot           =   obj.S_dot_links(:,interval_dimVel_i);
            %     XM              =   obj.XM_links(:,:,i);
            %     i_p             =   obj.parent_link(i);
            %     if i_p ~= 0
            %         acc_lib(:,i)    =   XM*acc_lib(:,i_p) + S_dot*vq;
            %     else
            %         acc_lib(:,i)    =   XM*acc_0 + S_dot*vq;
            %     end
            %     f_lib(:,i)      =   obj.I_s(:,:,i)*acc_lib(:,i) + vecX6DF(vel)*obj.I_s(:,:,i)*vel;
            % end
            
            % for j = 1:obj.numLinks
            %     i           =   obj.numLinks + 1 - j;
            %     interval_dimVel_i = obj.dof_interval_links(1,i):obj.dof_interval_links(2,i);
            %     wrench(interval_dimVel_i)   =   obj.S_links(:,interval_dimVel_i)'*f_lib(:,i);
            %     i_p = obj.parent_link(i);
            %     if i_p ~= 0
            %         XF              =   obj.XF_links(:,:,i);
            %         f_lib(:,i_p)    =   f_lib(:,i_p) + XF*f_lib(:,i);
            %     end
            % end
            
            for i = 1:obj.numLinks
                interval_dimVel_i = obj.dof_interval_links(1,i):obj.dof_interval_links(2,i);
                vel             =   obj.vel_links(:,i);
                i_p             =   obj.parent_link(i);
                if i_p ~= 0
                    acc_lib(:,i)    =   obj.XM_links(:,:,i)*acc_lib(:,i_p) + obj.S_dot_links(:,interval_dimVel_i)*obj.x_vel(interval_dimVel_i);
                else
                    acc_lib(:,i)    =   obj.XM_links(:,:,i)*acc_0 + obj.S_dot_links(:,interval_dimVel_i)*obj.x_vel(interval_dimVel_i);
                end
                f_lib(:,i)      =   obj.I_s(:,:,i)*acc_lib(:,i) + vecX6DF(vel)*obj.I_s(:,:,i)*vel;
            end
            
            for j = 1:obj.numLinks
                i           =   obj.numLinks + 1 - j;
                interval_dimVel_i = obj.dof_interval_links(1,i):obj.dof_interval_links(2,i);
                wrench(interval_dimVel_i)   =   obj.S_links(:,interval_dimVel_i)'*f_lib(:,i);
                i_p = obj.parent_link(i);
                if i_p ~= 0
                    f_lib(:,i_p)    =   f_lib(:,i_p) + obj.XF_links(:,:,i)*f_lib(:,i);
                end
            end
            
            obj.C = wrench;
            C = obj.C;
        end
        % function G
        function G = updateG(obj)
            % two versions here, use cell array or use regular array, see which one is faster
            
            % f_lib = cell(obj.numLinks, 1);
            % acc_lib = cell(obj.numLinks, 1);
            % wrench = cell(obj.numLinks, 1);
            
            % acc_0 = [0; 0; 0; 0; 0; obj.g];
            
            % % for i = 1:obj.numLinks
            % %     XM          =   obj.XM_links(:,:,i);
            % %     i_p             =   obj.parent_link(i);
            % %     if i_p ~= 0
            % %         acc_lib{i}  =   XM*acc_lib{i_p};
            % %     else
            % %         acc_lib{i}  =   XM*acc_0;
            % %     end
            % %     f_lib{i}    =   obj.I_s(:,:,i)*acc_lib{i};
            % % end
            
            % % for j = 1:obj.numLinks
            % %     i           =   obj.numLinks + 1 - j;
            % %     interval_dimVel_i = obj.dof_interval_links(1,i):obj.dof_interval_links(2,i);
            % %     wrench{i}   =   obj.S_links(:,interval_dimVel_i)'*f_lib{i};
            % %     i_p = obj.parent_link(i);
            % %     if i_p ~= 0
            % %         XF          =   obj.XF_links(:,:,i);
            % %         f_lib{i_p}  =   f_lib{i_p} + XF*f_lib{i};
            % %     end
            % % end
            
            % for i = 1:obj.numLinks
            %     i_p             =   obj.parent_link(i);
            %     if i_p ~= 0
            %         acc_lib{i}  =   obj.XM_links(:,:,i)*acc_lib{i_p};
            %     else
            %         acc_lib{i}  =   obj.XM_links(:,:,i)*acc_0;
            %     end
            %     f_lib{i}    =   obj.I_s(:,:,i)*acc_lib{i};
            % end
            
            % for j = 1:obj.numLinks
            %     i           =   obj.numLinks + 1 - j;
            %     interval_dimVel_i = obj.dof_interval_links(1,i):obj.dof_interval_links(2,i);
            %     wrench{i}   =   obj.S_links(:,interval_dimVel_i)'*f_lib{i};
            %     i_p = obj.parent_link(i);
            %     if i_p ~= 0
            %         f_lib{i_p}  =   f_lib{i_p} + obj.XF_links(:,:,i)*f_lib{i};
            %     end
            % end
            
            % obj.G = cell2mat(wrench);
            % G = obj.G;

            
            
            f_lib = zeros(6, obj.numLinks);
            acc_lib = zeros(6, obj.numLinks);
            wrench = zeros(obj.dimVelocity, 1);
            
            acc_0 = [0; 0; 0; 0; 0; obj.g];
            
            % for i = 1:obj.numLinks
            %     XM          =   obj.XM_links(:,:,i);
            %     i_p             =   obj.parent_link(i);
            %     if i_p ~= 0
            %         acc_lib(:,i)    =   XM*acc_lib(:,i_p);
            %     else
            %         acc_lib(:,i)    =   XM*acc_0;
            %     end
            %     f_lib(:,i)  =   obj.I_s(:,:,i)*acc_lib(:,i);
            % end
            
            % for j = 1:obj.numLinks
            %     i           =   obj.numLinks + 1 - j;
            %     interval_dimVel_i = obj.dof_interval_links(1,i):obj.dof_interval_links(2,i);
            %     wrench(interval_dimVel_i)   =   obj.S_links(:,interval_dimVel_i)'*f_lib(:,i);
            %     i_p = obj.parent_link(i);
            %     if i_p ~= 0
            %         XF              =   obj.XF_links(:,:,i);
            %         f_lib(:,i_p)    =   f_lib(:,i_p) + XF*f_lib(:,i);
            %     end
            % end
            
            for i = 1:obj.numLinks
                i_p             =   obj.parent_link(i);
                if i_p ~= 0
                    acc_lib(:,i)    =   obj.XM_links(:,:,i)*acc_lib(:,i_p);
                else
                    acc_lib(:,i)    =   obj.XM_links(:,:,i)*acc_0;
                end
                f_lib(:,i)  =   obj.I_s(:,:,i)*acc_lib(:,i);
            end
            
            for j = 1:obj.numLinks
                i           =   obj.numLinks + 1 - j;
                interval_dimVel_i = obj.dof_interval_links(1,i):obj.dof_interval_links(2,i);
                wrench(interval_dimVel_i)   =   obj.S_links(:,interval_dimVel_i)'*f_lib(:,i);
                i_p = obj.parent_link(i);
                if i_p ~= 0
                    f_lib(:,i_p)    =   f_lib(:,i_p) + obj.XF_links(:,:,i)*f_lib(:,i);
                end
            end
            
            obj.G = wrench;
            G = obj.G;
        end

        
        function updateCG(obj)
            % two versions here, use cell array or use regular array, see which one is faster
            
            % f_C_lib = cell(obj.numLinks, 1);
            % f_G_lib = cell(obj.numLinks, 1);
            % acc_C_lib = cell(obj.numLinks, 1);
            % acc_G_lib = cell(obj.numLinks, 1);
            % wrench_C = cell(obj.numLinks, 1);
            % wrench_G = cell(obj.numLinks, 1);
            
            % acc_C_0 = zeros(6,1);
            % acc_G_0 = [0;0;0;0;0;obj.g];
            
            % % for i = 1:obj.numLinks
            % %     interval_dimVel_i = obj.dof_interval_links(1,i):obj.dof_interval_links(2,i);
            % %     vq              =   obj.x_vel(interval_dimVel_i);
            % %     vel             =   obj.vel_links(:,i);
            % %     S_dot           =   obj.S_dot_links(:,interval_dimVel_i);
            % %     XM              =   obj.XM_links(:,:,i);
            % %     i_p             =   obj.parent_link(i);
            % %     if i_p ~= 0
            % %         acc_C_lib{i}    =   XM*acc_C_lib{i_p} + S_dot*vq;
            % %         acc_G_lib{i}    =   XM*acc_G_lib{i_p};
            % %     else
            % %         acc_C_lib{i}    =   XM*acc_C_0 + S_dot*vq;
            % %         acc_G_lib{i}    =   XM*acc_G_0;
            % %     end
            % %     f_C_lib{i}      =   obj.I_s(:,:,i)*acc_C_lib{i} + vecX6DF(vel)*obj.I_s(:,:,i)*vel;
            % %     f_G_lib{i}      =   obj.I_s(:,:,i)*acc_G_lib{i};
            % % end
            
            % % for j = 1:obj.numLinks
            % %     i           =   obj.numLinks + 1 - j;
            % %     interval_dimVel_i = obj.dof_interval_links(1,i):obj.dof_interval_links(2,i);
            % %     wrench_C{i}   =   obj.S_links(:,interval_dimVel_i)'*f_C_lib{i};
            % %     wrench_G{i}   =   obj.S_links(:,interval_dimVel_i)'*f_G_lib{i};
            % %     i_p = obj.parent_link(i);
            % %     if i_p ~= 0
            % %         XF          =   obj.XF_links(:,:,i);
            % %         f_C_lib{i_p}  =   f_C_lib{i_p} + XF*f_C_lib{i};
            % %         f_G_lib{i_p}  =   f_G_lib{i_p} + XF*f_G_lib{i};
            % %     end
            % % end
            
            
            % for i = 1:obj.numLinks
            %     interval_dimVel_i = obj.dof_interval_links(1,i):obj.dof_interval_links(2,i);
            %     I_s_i           =   obj.I_s(:,:,i);
            %     vel             =   obj.vel_links(:,i);
            %     XM              =   obj.XM_links(:,:,i);
            %     i_p             =   obj.parent_link(i);
            %     if i_p ~= 0
            %         acc_C_lib{i}    =   XM*acc_C_lib{i_p} + obj.S_dot_links(:,interval_dimVel_i)*obj.x_vel(interval_dimVel_i);
            %         acc_G_lib{i}    =   XM*acc_G_lib{i_p};
            %     else
            %         acc_C_lib{i}    =   XM*acc_C_0 + obj.S_dot_links(:,interval_dimVel_i)*obj.x_vel(interval_dimVel_i);
            %         acc_G_lib{i}    =   XM*acc_G_0;
            %     end
            %     f_C_lib{i}      =   I_s_i*acc_C_lib{i} + vecX6DF(vel)*I_s_i*vel;
            %     f_G_lib{i}      =   I_s_i*acc_G_lib{i};
            % end
            
            % for j = 1:obj.numLinks
            %     i           =   obj.numLinks + 1 - j;
            %     interval_dimVel_i = obj.dof_interval_links(1,i):obj.dof_interval_links(2,i);
            %     wrench_C{i}   =   obj.S_links(:,interval_dimVel_i)'*f_C_lib{i};
            %     wrench_G{i}   =   obj.S_links(:,interval_dimVel_i)'*f_G_lib{i};
            %     i_p = obj.parent_link(i);
            %     if i_p ~= 0
            %         XF          =   obj.XF_links(:,:,i);
            %         f_C_lib{i_p}  =   f_C_lib{i_p} + XF*f_C_lib{i};
            %         f_G_lib{i_p}  =   f_G_lib{i_p} + XF*f_G_lib{i};
            %     end
            % end
            
            % obj.C = cell2mat(wrench_C);
            % obj.G = cell2mat(wrench_G);

            
            
            f_C_lib = zeros(6, obj.numLinks);
            f_G_lib = zeros(6, obj.numLinks);
            acc_C_lib = zeros(6, obj.numLinks);
            acc_G_lib = zeros(6, obj.numLinks);
            wrench_C = zeros(obj.dimVelocity, 1);
            wrench_G = zeros(obj.dimVelocity, 1);
            
            acc_C_0 = [0; 0; 0; 0; 0; 0];
            acc_G_0 = [0; 0; 0; 0; 0; obj.g];
            
            % for i = 1:obj.numLinks
            %     interval_dimVel_i = obj.dof_interval_links(1,i):obj.dof_interval_links(2,i);
            %     vq              =   obj.x_vel(interval_dimVel_i);
            %     vel             =   obj.vel_links(:,i);
            %     S_dot           =   obj.S_dot_links(:,interval_dimVel_i);
            %     XM              =   obj.XM_links(:,:,i);
            %     i_p             =   obj.parent_link(i);
            %     if i_p ~= 0
            %         acc_C_lib(:,i)    =   XM*acc_C_lib(:,i_p) + S_dot*vq;
            %         acc_G_lib(:,i)    =   XM*acc_G_lib(:,i_p);
            %     else
            %         acc_C_lib(:,i)    =   XM*acc_C_0 + S_dot*vq;
            %         acc_G_lib(:,i)    =   XM*acc_G_0;
            %     end
            %     f_C_lib(:,i)    =   obj.I_s(:,:,i)*acc_C_lib(:,i) + vecX6DF(vel)*obj.I_s(:,:,i)*vel;
            %     f_G_lib(:,i)    =   obj.I_s(:,:,i)*acc_G_lib(:,i);
            % end
            
            % for j = 1:obj.numLinks
            %     i           =   obj.numLinks + 1 - j;
            %     interval_dimVel_i = obj.dof_interval_links(1,i):obj.dof_interval_links(2,i);
            %     wrench_C(interval_dimVel_i)   =   obj.S_links(:,interval_dimVel_i)'*f_C_lib(:,i);
            %     wrench_G(interval_dimVel_i)   =   obj.S_links(:,interval_dimVel_i)'*f_G_lib(:,i);
            %     i_p = obj.parent_link(i);
            %     if i_p ~= 0
            %         XF              =   obj.XF_links(:,:,i);
            %         f_C_lib(:,i_p)    =   f_C_lib(:,i_p) + XF*f_C_lib(:,i);
            %         f_G_lib(:,i_p)    =   f_G_lib(:,i_p) + XF*f_G_lib(:,i);
            %     end
            % end
            

            for i = 1:obj.numLinks
                interval_dimVel_i = obj.dof_interval_links(1,i):obj.dof_interval_links(2,i);
                I_s_i           =   obj.I_s(:,:,i);
                vel             =   obj.vel_links(:,i);
                XM              =   obj.XM_links(:,:,i);
                i_p             =   obj.parent_link(i);
                if i_p ~= 0
                    acc_C_lib(:,i)    =   XM*acc_C_lib(:,i_p) + obj.S_dot_links(:,interval_dimVel_i)*obj.x_vel(interval_dimVel_i);
                    acc_G_lib(:,i)    =   XM*acc_G_lib(:,i_p);
                else
                    acc_C_lib(:,i)    =   XM*acc_C_0 + obj.S_dot_links(:,interval_dimVel_i)*obj.x_vel(interval_dimVel_i);
                    acc_G_lib(:,i)    =   XM*acc_G_0;
                end
                f_C_lib(:,i)    =   I_s_i*acc_C_lib(:,i) + vecX6DF(vel)*I_s_i*vel;
                f_G_lib(:,i)    =   I_s_i*acc_G_lib(:,i);
            end
            
            for j = 1:obj.numLinks
                i           =   obj.numLinks + 1 - j;
                interval_dimVel_i = obj.dof_interval_links(1,i):obj.dof_interval_links(2,i);
                wrench_C(interval_dimVel_i)   =   obj.S_links(:,interval_dimVel_i)'*f_C_lib(:,i);
                wrench_G(interval_dimVel_i)   =   obj.S_links(:,interval_dimVel_i)'*f_G_lib(:,i);
                i_p = obj.parent_link(i);
                if i_p ~= 0
                    XF              =   obj.XF_links(:,:,i);
                    f_C_lib(:,i_p)    =   f_C_lib(:,i_p) + XF*f_C_lib(:,i);
                    f_G_lib(:,i_p)    =   f_G_lib(:,i_p) + XF*f_G_lib(:,i);
                end
            end
            
            obj.C = wrench_C;
            obj.G = wrench_G;
        end

        
        % function that computes the actuation mappings (cable forces, active joint torques and propeller forces)
        function updateActuationMapping(obj)
            % assumes that the system state is updated, i.e. obj.updateSystemState(x_pos, x_vel) is called

            % part 0: preparation work, generate link Jacobians in local frame
            if isempty(obj.cables)
                linkIndexMax_cable = 0;
            else
                linkIndexMax_cable = max(obj.cable_data_attachment_link_indices);
            end
            if isempty(obj.propellers)
                linkIndexMax_propeller = 0;
            else
                linkIndexMax_propeller = max(obj.propeller_data_link_indices);
            end
            linkIndexMax = max([linkIndexMax_cable, linkIndexMax_propeller]);
            % build a library of link Jacobians in the local frame first
            obj.LinkJacobianLibrary = zeros(6, obj.dimVelocity, linkIndexMax);
            JL          =   zeros(6, obj.dimVelocity);
            for i = 1:linkIndexMax
                interval_dimVel_i = obj.dof_interval_links(1,i):obj.dof_interval_links(2,i);
                p_i = obj.parent_link(i);
                if p_i ~= 0
                    % compute the Jacobian of link i
                    JL              =   obj.XM_links(:,:,i)*obj.LinkJacobianLibrary(:,:,p_i);
                    JL(:,interval_dimVel_i)... 
                                    =   obj.S_links(:,interval_dimVel_i);
                else
                    % compute the Jacobian of link i
                    JL(:,interval_dimVel_i)... 
                                    =   obj.S_links(:,interval_dimVel_i);
                end
                % JL is in link i's local frame
                obj.LinkJacobianLibrary(:,:,i) = JL;
            end



            % part 1: cable actuation mapping
            % kinematics based approach (investigate the relationship between the joint velocity and the cable velocity)
            obj.L = zeros(obj.numCables, obj.dimVelocity);
            % go through each cable, generate the cable attachment velocity Jacobian in local frame, then derive the cable's corresponding row in L
            for i = 1:obj.numCables
                attachment_index = obj.cables{i};
                num_attachment = length(attachment_index);
                Li = zeros(1, obj.dimVelocity);
                for j = 1:num_attachment
                    direction_vec_set = [];
                    current_attachment_idx = attachment_index(j);
                    current_attachment_link_idx = obj.cable_data_attachment_link_indices(current_attachment_idx);
                    if current_attachment_link_idx > 0
                        current_attachment = obj.cable_data_attachments(:,current_attachment_idx);
                        if j < num_attachment
                            subsequent_attachment_idx = attachment_index(j+1);
                            subsequent_attachment_link_idx = obj.cable_data_attachment_link_indices(subsequent_attachment_idx);
                            if subsequent_attachment_link_idx ~= current_attachment_link_idx
                                subsequent_attachment = obj.cable_data_attachments(:,subsequent_attachment_idx);
                                subsequent_attachment_in_current_frame = obj.pointCoordinate(subsequent_attachment_link_idx, subsequent_attachment, current_attachment_link_idx);
                                subsequent_to_current = current_attachment - subsequent_attachment_in_current_frame;
                                subsequent_to_current = subsequent_to_current / (norm(subsequent_to_current) + 1e-8);
                                direction_vec_set = [direction_vec_set, subsequent_to_current];
                            end
                        end
                        if j > 1
                            prior_attachment_idx = attachment_index(j-1);
                            prior_attachment_link_idx = obj.cable_data_attachment_link_indices(prior_attachment_idx);
                            if prior_attachment_link_idx ~= current_attachment_link_idx
                                prior_attachment = obj.cable_data_attachments(:,prior_attachment_idx);
                                prior_attachment_in_current_frame = obj.pointCoordinate(prior_attachment_link_idx, prior_attachment, current_attachment_link_idx);
                                prior_to_current = current_attachment - prior_attachment_in_current_frame;
                                prior_to_current = prior_to_current / (norm(prior_to_current) + 1e-8);
                                direction_vec_set = [direction_vec_set, prior_to_current];
                            end
                        end
                        num_direction_vec = size(direction_vec_set, 2);
                        if num_direction_vec > 0
                            current_link_root_path = obj.findRootPath(current_attachment_link_idx);
                            relevant_vel_dof_set = obj.findDofSet(current_link_root_path);
                            Jacobian_projector = [-vecX3D(current_attachment)  eye(3)];
                            Jacobian_relavent = obj.LinkJacobianLibrary(:,relevant_vel_dof_set,current_attachment_link_idx);
                            Jacobian_attachment = Jacobian_projector*Jacobian_relavent;
                            for k = 1:num_direction_vec
                                Li(relevant_vel_dof_set) = Li(relevant_vel_dof_set) + direction_vec_set(:,k)'*Jacobian_attachment;
                            end
                        end
                    end
                end
                
%                 numSegments = length(attachment_index) - 1;
%                 tmp_index = attachment_index(1);
%                 prev_link_index = obj.cable_data_attachment_link_indices(tmp_index);
%                 prev_attachment = obj.cable_data_attachments(:,tmp_index);
%                 Li = zeros(1, obj.dimVelocity);
%                 for j = 1:numSegments
%                     tmp_index = attachment_index(j+1);
%                     new_link_index = obj.cable_data_attachment_link_indices(tmp_index);
%                     new_attachment = obj.cable_data_attachments(:,tmp_index);
%                     
%                     if new_link_index ~= prev_link_index
%                         % if new_link_index > prev_link_index
%                         %     distal_link_index = new_link_index;
%                         %     proximal_link_index = prev_link_index;
%                         %     distal_attachment = new_attachment;
%                         %     proximal_attachment = prev_attachment;
%                         % else
%                         %     distal_link_index = prev_link_index;
%                         %     proximal_link_index = new_link_index;
%                         %     distal_attachment = prev_attachment;
%                         %     proximal_attachment = new_attachment;
%                         % end
%                         % 
%                         % segment_dof_interval = sum(obj.dimVelocityEachLink(1:proximal_link_index))+1 : sum(obj.dimVelocityEachLink(1:proximal_link_index))+sum(obj.dimVelocityEachLink(proximal_link_index+1:distal_link_index));
%                         % proximal_attachment = obj.pointCoordinate(proximal_link_index, proximal_attachment, distal_link_index);
%                         % segment_proximal_to_distal = distal_attachment - proximal_attachment;
%                         % Jacobian_projector = [-vecX3D(distal_attachment)  eye(3)];
%                         % Jacobian_relavent = obj.LinkJacobianLibrary(:,segment_dof_interval,distal_link_index);
%                         % Jacobian_attachment = Jacobian_projector*Jacobian_relavent;
%                         % Li(segment_dof_interval) = Li(segment_dof_interval) + segment_proximal_to_distal'*Jacobian_attachment/norm(segment_proximal_to_distal);
%                         
%                         
%                         path_prev_link_to_new_link = obj.findBranchOfIntervalPath(prev_link_index, new_link_index);
%                         relevant_vel_dof_set = obj.findDofSet(path_prev_link_to_new_link);
%                         
%                         prev_attachment_in_new_frame = obj.pointCoordinate(prev_link_index, prev_attachment, new_link_index);
%                         segment_prev_to_new_in_new_frame = new_attachment - prev_attachment_in_new_frame;
%                         Jacobian_projector = [-vecX3D(new_attachment)  eye(3)];
%                         Jacobian_relavent = obj.LinkJacobianLibrary(:,relevant_vel_dof_set,new_link_index);
%                         Jacobian_attachment = Jacobian_projector*Jacobian_relavent;
%                         Li(relevant_vel_dof_set) = Li(relevant_vel_dof_set) + segment_prev_to_new_in_new_frame'*Jacobian_attachment/norm(segment_prev_to_new_in_new_frame);
%                         
%                     end
%                     
%                     prev_link_index = new_link_index;
%                     prev_attachment = new_attachment;
%                 end
                obj.L(i,:) = Li;
            end
            
            % % this version of cable Jacobian works now
            % % force based approach (investigate the relationship between the joint wrench and the cable force)
            % L_t = zeros(obj.dimVelocity, obj.numCables);
            % % go through each cable, generate the cable attachment velocity Jacobian in local frame, then derive the cable's corresponding row in L
            % for i = 1:obj.numCables
            %     attachment_index = obj.cables{i};
            %     L_ti = zeros(obj.dimVelocity, 1);
            %     for j = 1:length(attachment_index)
            %         current_attachment_index = attachment_index(j);
            %         current_link_index = obj.cable_data_attachment_link_indices(current_attachment_index);
            %         current_attachment = obj.cable_data_attachments(:,current_attachment_index);
            %         if current_link_index == 0
            %             Jacobian_attachment = zeros(3, obj.dimVelocity);
            %         else
            %             Jacobian_projector = [-vecX3D(current_attachment)  eye(3)];
            %             Jacobian_relavent = obj.LinkJacobianLibrary(:,:,current_link_index);
            %             Jacobian_attachment = Jacobian_projector*Jacobian_relavent;
            %         end
            %         if j == 1
            %             next_attachment = obj.cable_data_attachments(:,attachment_index(j+1));
            %             next_link_index = obj.cable_data_attachment_link_indices(attachment_index(j+1));
            %             next_attachment = obj.pointCoordinate(next_link_index, next_attachment, current_link_index);
            %             segment_next = next_attachment - current_attachment;
            %             L_ti = L_ti + Jacobian_attachment'*(segment_next/norm(segment_next));
            %         elseif j == length(attachment_index)
            %             prev_attachment = obj.cable_data_attachments(:,attachment_index(j-1));
            %             prev_link_index = obj.cable_data_attachment_link_indices(attachment_index(j-1));
            %             prev_attachment = obj.pointCoordinate(prev_link_index, prev_attachment, current_link_index);
            %             segment_prev = prev_attachment - current_attachment;
            %             L_ti = L_ti + Jacobian_attachment'*(segment_prev/norm(segment_prev));
            %         else
            %             prev_attachment = obj.cable_data_attachments(:,attachment_index(j-1));
            %             prev_link_index = obj.cable_data_attachment_link_indices(attachment_index(j-1));
            %             prev_attachment = obj.pointCoordinate(prev_link_index, prev_attachment, current_link_index);
            %             next_attachment = obj.cable_data_attachments(:,attachment_index(j+1));
            %             next_link_index = obj.cable_data_attachment_link_indices(attachment_index(j+1));
            %             next_attachment = obj.pointCoordinate(next_link_index, next_attachment, current_link_index);
            %             segment_next = next_attachment - current_attachment;
            %             segment_prev = prev_attachment - current_attachment;
            %             L_ti = L_ti + Jacobian_attachment'*(segment_prev/norm(segment_prev)+segment_next/norm(segment_next));
            %         end
            %     end
            %     L_t(:,i) = L_ti;
            % end
            % obj.L = -L_t';

            
            % part 2: active joint actuation mapping
            % since this is joint pose independent, it is defined in the constructor and no need to update in each control cycle

            
            % % part 3: propeller actuation mapping (pure force)
            % T_v = zeros(2*obj.numPropellers, obj.dimVelocity);
            % % go through each propeller, generate the propeller velocity Jacobian along the given direction in local frame, then derive the propeller's corresponding row in T
            % for i = 1:obj.numPropellers
            %     install_link = obj.propeller_data_link_indices(i);
            %     install_point = obj.propeller_data_locations(i);
            %     install_direction = obj.propeller_data_directions(i);
                
            %     % Ti_v = zeros(2, obj.dimVelocity);
            %     Jacobian_attachment = xltMMat(install_point)*obj.LinkJacobianLibrary(:,:,install_link);
            %     Ti_v = [install_direction'*Jacobian_attachment(4:6,:);
            %             install_direction'*Jacobian_attachment(1:3,:)];

            %     T_v(2*i-1:2*i,:) = Ti_v;
            % end

            % part 3: propeller actuation mapping (force and torque)
            obj.T = zeros(obj.dimVelocity, obj.numPropellers);
            % go through each propeller, generate the propeller velocity Jacobian along the given direction in local frame, then derive the propeller's corresponding row in T
            for idx_i = 1:obj.numPropellers
                i = obj.propellers(idx_i);
                install_link = obj.propeller_data_link_indices(i);
                install_point = obj.propeller_data_locations(:, i);
                force_direction = -obj.propeller_data_directions(:, i); % force direction
                torque_coefficient = obj.propeller_data_c(i);

                link_force_i = [torque_coefficient*force_direction; force_direction];
                Jacobian_attachment = xltMMat(install_point)*obj.LinkJacobianLibrary(:,:,install_link);

                obj.T(:,i) = Jacobian_attachment'*link_force_i;
            end



            obj.A = [obj.W, obj.T];
            obj.AM = [-obj.L', obj.A];
        end

        
        % function that computes the actuation mapping matrix's sensitivity to reconfig variable (cable forces, active joint torques and propeller forces)
        function [cable_col_sensitivity_matrices,thruster_col_base_sensitivity_matrices,thruster_col_direction_sensitivity_matrices] = updateActuationMappingSensitivity(obj, cable_reconfigurability, thruster_base_reconfigurability, thruster_direction_reconfigurability)
            % assumes that the system state is updated, i.e. obj.updateSystemState(x_pos, x_vel) is called
            % assumes that the actuation mapping is updated, i.e. obj.updateActuationMapping() is called.

            % inputs thruster_base_reconfigurability and thruster_direction_reconfigurability are logical vectors with the length of numPropellers
            %   each element is assigned as 'true' if the corresponding item (base or direction) is reconfigurable, false otherwise
            % input cable_reconfigurability is a cell array with the length of numCables
            %   each element is assigned with a list of reconfigurable attachment index of the corresponding cable, the list can be empty if there are no reconfigurable attachments for the corresponding cable

            % part 0.1: link Jacobians are already generated in updateActuationMapping() function

            % part 0.2: initialization
            cable_col_sensitivity_matrices = cell(obj.numCables, 1);
            thruster_col_base_sensitivity_matrices = cell(obj.numPropellers, 1);
            thruster_col_direction_sensitivity_matrices = cell(obj.numPropellers, 1);

            % part 1: cable actuation mapping
            % go through each cable, generate a sensitivity matrix for each cable column w.r.t. reconfigurable attachment coordinates in the (attachment) local frame
            for i = 1:obj.numCables
                reconfigurable_attachment_index_set = cable_reconfigurability{i};
                num_reconfigurability_i = length(reconfigurable_attachment_index_set);
                sensitivity_matrix_cable_i = zeros(obj.dimVelocity, 3*num_reconfigurability_i);
                attachment_index_set = obj.cables{i};

                for j = 1:num_reconfigurability_i

                    index_set_j = 3*(j-1)+1:3*j;
                    % extract info on the reconfigurable attachment point (Ar)
                    Ar_index = reconfigurable_attachment_index_set(j);
                    Ar_link_index = obj.cable_data_attachment_link_indices(Ar_index);
                    Ar = obj.cable_data_attachments(:,Ar_index);
                    % extract info on the Jacobian
                    if Ar_link_index == 0
                        J_Ar = zeros(3, obj.dimVelocity);
                        J_Ar_dArx = zeros(3, obj.dimVelocity);
                        J_Ar_dAry = zeros(3, obj.dimVelocity);
                        J_Ar_dArz = zeros(3, obj.dimVelocity);

                        Ar_link_quat = [1;0;0;0];
                    else
                        Jacobian_relavent = obj.LinkJacobianLibrary(:,:,Ar_link_index);
                        Jacobian_projector = [-vecX3D(Ar)  eye(3)];
                        J_Ar = Jacobian_projector*Jacobian_relavent;

                        J_Ar_dArx = [-vecX3D([1,0,0])  zeros(3,3)]*Jacobian_relavent;
                        J_Ar_dAry = [-vecX3D([0,1,0])  zeros(3,3)]*Jacobian_relavent;
                        J_Ar_dArz = [-vecX3D([0,0,1])  zeros(3,3)]*Jacobian_relavent;

                        Ar_link_quat = obj.quat_links(:,Ar_link_index);
                    end

                    k = find(attachment_index_set==Ar_index);
                    if k == 1
                        % step 0: extract info on the next attachment point (An)
                        An = obj.cable_data_attachments(:,attachment_index_set(k+1));
                        An_link_index = obj.cable_data_attachment_link_indices(attachment_index_set(k+1));
                        % extract info on the Jacobian
                        if An_link_index == 0
                            J_An = zeros(3, obj.dimVelocity);
                            J_An_dArx = zeros(3, obj.dimVelocity);
                            J_An_dAry = zeros(3, obj.dimVelocity);
                            J_An_dArz = zeros(3, obj.dimVelocity);

                            An_link_quat = [1;0;0;0];
                        else
                            Jacobian_relavent = obj.LinkJacobianLibrary(:,:,An_link_index);
                            Jacobian_projector = [-vecX3D(An)  eye(3)];
                            J_An = Jacobian_projector*Jacobian_relavent;

                            J_An_dArx = zeros(3, obj.dimVelocity);
                            J_An_dAry = zeros(3, obj.dimVelocity);
                            J_An_dArz = zeros(3, obj.dimVelocity);

                            An_link_quat = obj.quat_links(:,An_link_index);
                        end

                        % step 1: process the current attachment point (the reconfigurable attachment point)
                        An_in_r = obj.pointCoordinate(An_link_index, An, Ar_link_index);
                        % the cable segment connecting the current reconfigurable attachment point and the subsequent attachment point
                        lrn = An_in_r - Ar;
                        % Jacobian of the cable segment (lrn) w.r.t. the reconfigurable attachment point coordinate
                        lrn_dAr = -eye(3);
                        % the normalized cable segment
                        lrn_norm = norm(lrn);
                        lrn_hat = lrn/lrn_norm;
                        % partial derivative of normalized cable segment w.r.t. lrn(1), lrn(2) and lrn(3)
                        lrn_hat_dlrnx = [1;0;0]/lrn_norm - lrn/(lrn_norm^3)*lrn(1);
                        lrn_hat_dlrny = [0;1;0]/lrn_norm - lrn/(lrn_norm^3)*lrn(2);
                        lrn_hat_dlrnz = [0;0;1]/lrn_norm - lrn/(lrn_norm^3)*lrn(3);
                        % Jacobian of the normalized cable segment vector (lrn_hat) w.r.t. lrn(1), lrn(2) and lrn(3)
                        lrn_hat_dlrn = [lrn_hat_dlrnx, lrn_hat_dlrny, lrn_hat_dlrnz];
                        % Jacobian of the normalized cable segment vector (lrn_hat) w.r.t. the reconfigurable attachment point coordinate
                        lrn_hat_dAr = lrn_hat_dlrn*lrn_dAr;
                        % Jacobian of the resulting wrench w.r.t. the reconfigurable attachment point coordinate
                        w_dAr = J_Ar'*lrn_hat_dAr + [J_Ar_dArx'*lrn_hat,J_Ar_dAry'*lrn_hat,J_Ar_dArz'*lrn_hat];
                        sensitivity_matrix_cable_i(:,index_set_j) = sensitivity_matrix_cable_i(:,index_set_j) + w_dAr;
                    
                        % step 2: process the next adjacent attachment point
                        % rotation matrix from the Ar link to An link
                        quat_delta = QuaternionMultiplication(QuaternionConjugate(Ar_link_quat), An_link_quat);
                        R_r_to_n = QuaternionToRotationMatrix(quat_delta);
                        Ar_in_n = obj.pointCoordinate(Ar_link_index, Ar, An_link_index);
                        % the cable segment connecting the current reconfigurable attachment point and the subsequent attachment point
                        lnr = Ar_in_n - An;
                        % Jacobian of the cable segment (lnr) w.r.t. the reconfigurable attachment point coordinate
                        lnr_dAr = R_r_to_n;
                        % the normalized cable segment
                        lnr_norm = norm(lnr);
                        lnr_hat = lnr/lnr_norm;
                        % partial derivative of normalized cable segment w.r.t. lnr(1), lnr(2) and lnr(3)
                        lnr_hat_dlnrx = [1;0;0]/lnr_norm - lnr/(lnr_norm^3)*lnr(1);
                        lnr_hat_dlnry = [0;1;0]/lnr_norm - lnr/(lnr_norm^3)*lnr(2);
                        lnr_hat_dlnrz = [0;0;1]/lnr_norm - lnr/(lnr_norm^3)*lnr(3);
                        % Jacobian of the normalized cable segment vector (lnr_hat) w.r.t. lnr(1), lnr(2) and lnr(3)
                        lnr_hat_dlnr = [lnr_hat_dlnrx, lnr_hat_dlnry, lnr_hat_dlnrz];
                        % Jacobian of the normalized cable segment vector (lnr_hat) w.r.t. the reconfigurable attachment point coordinate
                        lnr_hat_dAr = lnr_hat_dlnr*lnr_dAr;
                        % Jacobian of the resulting wrench w.r.t. the reconfigurable attachment point coordinate
                        w_dAr = J_An'*lnr_hat_dAr + [J_An_dArx'*lnr_hat,J_An_dAry'*lnr_hat,J_An_dArz'*lnr_hat];
                        sensitivity_matrix_cable_i(:,index_set_j) = sensitivity_matrix_cable_i(:,index_set_j) + w_dAr;
                    
                    elseif k == length(attachment_index_set)

                        % step 0: extract info on the previous attachment point (Ap)
                        Ap = obj.cable_data_attachments(:,attachment_index_set(k-1));
                        Ap_link_index = obj.cable_data_attachment_link_indices(attachment_index_set(k-1));
                        % extract info on the Jacobian
                        if Ap_link_index == 0
                            J_Ap = zeros(3, obj.dimVelocity);
                            J_Ap_dArx = zeros(3, obj.dimVelocity);
                            J_Ap_dAry = zeros(3, obj.dimVelocity);
                            J_Ap_dArz = zeros(3, obj.dimVelocity);

                            Ap_link_quat = [1;0;0;0];
                        else
                            Jacobian_relavent = obj.LinkJacobianLibrary(:,:,Ap_link_index);
                            Jacobian_projector = [-vecX3D(Ap)  eye(3)];
                            J_Ap = Jacobian_projector*Jacobian_relavent;

                            J_Ap_dArx = zeros(3, obj.dimVelocity);
                            J_Ap_dAry = zeros(3, obj.dimVelocity);
                            J_Ap_dArz = zeros(3, obj.dimVelocity);

                            Ap_link_quat = obj.quat_links(:,Ap_link_index);
                        end

                        % step 1: process the current attachment point (the reconfigurable attachment point)
                        Ap_in_r = obj.pointCoordinate(Ap_link_index, Ap, Ar_link_index);
                        % the cable segment connecting the current reconfigurable attachment point and the previous attachment point
                        lrp = Ap_in_r - Ar;
                        % Jacobian of the cable segment (lrp) w.r.t. the reconfigurable attachment point coordinate
                        lrp_dAr = -eye(3);
                        % the normalized cable segment
                        lrp_norm = norm(lrp);
                        lrp_hat = lrp/lrp_norm;
                        % partial derivative of normalized cable segment w.r.t. lrp(1), lrp(2) and lrp(3)
                        lrp_hat_dlrpx = [1;0;0]/lrp_norm - lrp/(lrp_norm^3)*lrp(1);
                        lrp_hat_dlrpy = [0;1;0]/lrp_norm - lrp/(lrp_norm^3)*lrp(2);
                        lrp_hat_dlrpz = [0;0;1]/lrp_norm - lrp/(lrp_norm^3)*lrp(3);
                        % Jacobian of the normalized cable segment vector (lrp_hat) w.r.t. lrp(1), lrp(2) and lrp(3)
                        lrp_hat_dlrp = [lrp_hat_dlrpx, lrp_hat_dlrpy, lrp_hat_dlrpz];
                        % Jacobian of the normalized cable segment vector (lrp_hat) w.r.t. the reconfigurable attachment point coordinate
                        lrp_hat_dAr = lrp_hat_dlrp*lrp_dAr;
                        % Jacobian of the resulting wrench w.r.t. the reconfigurable attachment point coordinate
                        w_dAr = J_Ar'*lrp_hat_dAr + [J_Ar_dArx'*lrp_hat,J_Ar_dAry'*lrp_hat,J_Ar_dArz'*lrp_hat];
                        sensitivity_matrix_cable_i(:,index_set_j) = sensitivity_matrix_cable_i(:,index_set_j) + w_dAr;
                    
                        % step 2: process the previous adjacent attachment point
                        % rotation matrix from the Ar link to Ap link
                        quat_delta = QuaternionMultiplication(QuaternionConjugate(Ar_link_quat), Ap_link_quat);
                        R_r_to_p = QuaternionToRotationMatrix(quat_delta);
                        Ar_in_p = obj.pointCoordinate(Ar_link_index, Ar, Ap_link_index);
                        % the cable segment connecting the current reconfigurable attachment point and the previous attachment point
                        lpr = Ar_in_p - Ap;
                        % Jacobian of the cable segment (lpr) w.r.t. the reconfigurable attachment point coordinate
                        lpr_dAr = R_r_to_p;
                        % the normalized cable segment
                        lpr_norm = norm(lpr);
                        lpr_hat = lpr/lpr_norm;
                        % partial derivative of normalized cable segment w.r.t. lpr(1), lpr(2) and lpr(3)
                        lpr_hat_dlprx = [1;0;0]/lpr_norm - lpr/(lpr_norm^3)*lpr(1);
                        lpr_hat_dlpry = [0;1;0]/lpr_norm - lpr/(lpr_norm^3)*lpr(2);
                        lpr_hat_dlprz = [0;0;1]/lpr_norm - lpr/(lpr_norm^3)*lpr(3);
                        % Jacobian of the normalized cable segment vector (lpr_hat) w.r.t. lpr(1), lpr(2) and lpr(3)
                        lpr_hat_dlpr = [lpr_hat_dlprx, lpr_hat_dlpry, lpr_hat_dlprz];
                        % Jacobian of the normalized cable segment vector (lpr_hat) w.r.t. the reconfigurable attachment point coordinate
                        lpr_hat_dAr = lpr_hat_dlpr*lpr_dAr;
                        % Jacobian of the resulting wrench w.r.t. the reconfigurable attachment point coordinate
                        w_dAr = J_Ap'*lpr_hat_dAr + [J_Ap_dArx'*lpr_hat,J_Ap_dAry'*lpr_hat,J_Ap_dArz'*lpr_hat];
                        sensitivity_matrix_cable_i(:,index_set_j) = sensitivity_matrix_cable_i(:,index_set_j) + w_dAr;
                    
                    else
                        
                        % step 0.1: extract info on the next attachment point (An)
                        An = obj.cable_data_attachments(:,attachment_index_set(k+1));
                        An_link_index = obj.cable_data_attachment_link_indices(attachment_index_set(k+1));
                        % extract info on the Jacobian
                        if An_link_index == 0
                            J_An = zeros(3, obj.dimVelocity);
                            J_An_dArx = zeros(3, obj.dimVelocity);
                            J_An_dAry = zeros(3, obj.dimVelocity);
                            J_An_dArz = zeros(3, obj.dimVelocity);

                            An_link_quat = [1;0;0;0];
                        else
                            Jacobian_relavent = obj.LinkJacobianLibrary(:,:,An_link_index);
                            Jacobian_projector = [-vecX3D(An)  eye(3)];
                            J_An = Jacobian_projector*Jacobian_relavent;

                            J_An_dArx = zeros(3, obj.dimVelocity);
                            J_An_dAry = zeros(3, obj.dimVelocity);
                            J_An_dArz = zeros(3, obj.dimVelocity);

                            An_link_quat = obj.quat_links(:,An_link_index);
                        end

                        % step 0.2: extract info on the previous attachment point (Ap)
                        Ap = obj.cable_data_attachments(:,attachment_index_set(k-1));
                        Ap_link_index = obj.cable_data_attachment_link_indices(attachment_index_set(k-1));
                        % extract info on the Jacobian
                        if Ap_link_index == 0
                            J_Ap = zeros(3, obj.dimVelocity);
                            J_Ap_dArx = zeros(3, obj.dimVelocity);
                            J_Ap_dAry = zeros(3, obj.dimVelocity);
                            J_Ap_dArz = zeros(3, obj.dimVelocity);

                            Ap_link_quat = [1;0;0;0];
                        else
                            Jacobian_relavent = obj.LinkJacobianLibrary(:,:,Ap_link_index);
                            Jacobian_projector = [-vecX3D(Ap)  eye(3)];
                            J_Ap = Jacobian_projector*Jacobian_relavent;

                            J_Ap_dArx = zeros(3, obj.dimVelocity);
                            J_Ap_dAry = zeros(3, obj.dimVelocity);
                            J_Ap_dArz = zeros(3, obj.dimVelocity);

                            Ap_link_quat = obj.quat_links(:,Ap_link_index);
                        end

                        % step 1.1: process the current attachment point (the reconfigurable attachment point)
                        An_in_r = obj.pointCoordinate(An_link_index, An, Ar_link_index);
                        % the cable segment connecting the current reconfigurable attachment point and the subsequent attachment point
                        lrn = An_in_r - Ar;
                        % Jacobian of the cable segment (lrn) w.r.t. the reconfigurable attachment point coordinate
                        lrn_dAr = -eye(3);
                        % the normalized cable segment
                        lrn_norm = norm(lrn);
                        lrn_hat = lrn/lrn_norm;
                        % partial derivative of normalized cable segment w.r.t. lrn(1), lrn(2) and lrn(3)
                        lrn_hat_dlrnx = [1;0;0]/lrn_norm - lrn/(lrn_norm^3)*lrn(1);
                        lrn_hat_dlrny = [0;1;0]/lrn_norm - lrn/(lrn_norm^3)*lrn(2);
                        lrn_hat_dlrnz = [0;0;1]/lrn_norm - lrn/(lrn_norm^3)*lrn(3);
                        % Jacobian of the normalized cable segment vector (lrn_hat) w.r.t. lrn(1), lrn(2) and lrn(3)
                        lrn_hat_dlrn = [lrn_hat_dlrnx, lrn_hat_dlrny, lrn_hat_dlrnz];
                        % Jacobian of the normalized cable segment vector (lrn_hat) w.r.t. the reconfigurable attachment point coordinate
                        lrn_hat_dAr = lrn_hat_dlrn*lrn_dAr;
                        % Jacobian of the resulting wrench w.r.t. the reconfigurable attachment point coordinate
                        w_dAr = J_Ar'*lrn_hat_dAr + [J_Ar_dArx'*lrn_hat,J_Ar_dAry'*lrn_hat,J_Ar_dArz'*lrn_hat];
                        sensitivity_matrix_cable_i(:,index_set_j) = sensitivity_matrix_cable_i(:,index_set_j) + w_dAr;

                        % step 1.2: process the current attachment point (the reconfigurable attachment point)
                        Ap_in_r = obj.pointCoordinate(Ap_link_index, Ap, Ar_link_index);
                        % the cable segment connecting the current reconfigurable attachment point and the previous attachment point
                        lrp = Ap_in_r - Ar;
                        % Jacobian of the cable segment (lrp) w.r.t. the reconfigurable attachment point coordinate
                        lrp_dAr = -eye(3);
                        % the normalized cable segment
                        lrp_norm = norm(lrp);
                        lrp_hat = lrp/lrp_norm;
                        % partial derivative of normalized cable segment w.r.t. lrp(1), lrp(2) and lrp(3)
                        lrp_hat_dlrpx = [1;0;0]/lrp_norm - lrp/(lrp_norm^3)*lrp(1);
                        lrp_hat_dlrpy = [0;1;0]/lrp_norm - lrp/(lrp_norm^3)*lrp(2);
                        lrp_hat_dlrpz = [0;0;1]/lrp_norm - lrp/(lrp_norm^3)*lrp(3);
                        % Jacobian of the normalized cable segment vector (lrp_hat) w.r.t. lrp(1), lrp(2) and lrp(3)
                        lrp_hat_dlrp = [lrp_hat_dlrpx, lrp_hat_dlrpy, lrp_hat_dlrpz];
                        % Jacobian of the normalized cable segment vector (lrp_hat) w.r.t. the reconfigurable attachment point coordinate
                        lrp_hat_dAr = lrp_hat_dlrp*lrp_dAr;
                        % Jacobian of the resulting wrench w.r.t. the reconfigurable attachment point coordinate
                        w_dAr = J_Ar'*lrp_hat_dAr + [J_Ar_dArx'*lrp_hat,J_Ar_dAry'*lrp_hat,J_Ar_dArz'*lrp_hat];
                        sensitivity_matrix_cable_i(:,index_set_j) = sensitivity_matrix_cable_i(:,index_set_j) + w_dAr;
                    
                        % step 2.1: process the next adjacent attachment point
                        % rotation matrix from the Ar link to An link
                        quat_delta = QuaternionMultiplication(QuaternionConjugate(Ar_link_quat), An_link_quat);
                        R_r_to_n = QuaternionToRotationMatrix(quat_delta);
                        Ar_in_n = obj.pointCoordinate(Ar_link_index, Ar, An_link_index);
                        % the cable segment connecting the current reconfigurable attachment point and the subsequent attachment point
                        lnr = Ar_in_n - An;
                        % Jacobian of the cable segment (lnr) w.r.t. the reconfigurable attachment point coordinate
                        lnr_dAr = R_r_to_n;
                        % the normalized cable segment
                        lnr_norm = norm(lnr);
                        lnr_hat = lnr/lnr_norm;
                        % partial derivative of normalized cable segment w.r.t. lnr(1), lnr(2) and lnr(3)
                        lnr_hat_dlnrx = [1;0;0]/lnr_norm - lnr/(lnr_norm^3)*lnr(1);
                        lnr_hat_dlnry = [0;1;0]/lnr_norm - lnr/(lnr_norm^3)*lnr(2);
                        lnr_hat_dlnrz = [0;0;1]/lnr_norm - lnr/(lnr_norm^3)*lnr(3);
                        % Jacobian of the normalized cable segment vector (lnr_hat) w.r.t. lnr(1), lnr(2) and lnr(3)
                        lnr_hat_dlnr = [lnr_hat_dlnrx, lnr_hat_dlnry, lnr_hat_dlnrz];
                        % Jacobian of the normalized cable segment vector (lnr_hat) w.r.t. the reconfigurable attachment point coordinate
                        lnr_hat_dAr = lnr_hat_dlnr*lnr_dAr;
                        % Jacobian of the resulting wrench w.r.t. the reconfigurable attachment point coordinate
                        w_dAr = J_An'*lnr_hat_dAr + [J_An_dArx'*lnr_hat,J_An_dAry'*lnr_hat,J_An_dArz'*lnr_hat];
                        sensitivity_matrix_cable_i(:,index_set_j) = sensitivity_matrix_cable_i(:,index_set_j) + w_dAr;
                    
                        % step 2.2: process the previous adjacent attachment point
                        % rotation matrix from the Ar link to Ap link
                        quat_delta = QuaternionMultiplication(QuaternionConjugate(Ar_link_quat), Ap_link_quat);
                        R_r_to_p = QuaternionToRotationMatrix(quat_delta);
                        Ar_in_p = obj.pointCoordinate(Ar_link_index, Ar, Ap_link_index);
                        % the cable segment connecting the current reconfigurable attachment point and the previous attachment point
                        lpr = Ar_in_p - Ap;
                        % Jacobian of the cable segment (lpr) w.r.t. the reconfigurable attachment point coordinate
                        lpr_dAr = R_r_to_p;
                        % the normalized cable segment
                        lpr_norm = norm(lpr);
                        lpr_hat = lpr/lpr_norm;
                        % partial derivative of normalized cable segment w.r.t. lpr(1), lpr(2) and lpr(3)
                        lpr_hat_dlprx = [1;0;0]/lpr_norm - lpr/(lpr_norm^3)*lpr(1);
                        lpr_hat_dlpry = [0;1;0]/lpr_norm - lpr/(lpr_norm^3)*lpr(2);
                        lpr_hat_dlprz = [0;0;1]/lpr_norm - lpr/(lpr_norm^3)*lpr(3);
                        % Jacobian of the normalized cable segment vector (lpr_hat) w.r.t. lpr(1), lpr(2) and lpr(3)
                        lpr_hat_dlpr = [lpr_hat_dlprx, lpr_hat_dlpry, lpr_hat_dlprz];
                        % Jacobian of the normalized cable segment vector (lpr_hat) w.r.t. the reconfigurable attachment point coordinate
                        lpr_hat_dAr = lpr_hat_dlpr*lpr_dAr;
                        % Jacobian of the resulting wrench w.r.t. the reconfigurable attachment point coordinate
                        w_dAr = J_Ap'*lpr_hat_dAr + [J_Ap_dArx'*lpr_hat,J_Ap_dAry'*lpr_hat,J_Ap_dArz'*lpr_hat];
                        sensitivity_matrix_cable_i(:,index_set_j) = sensitivity_matrix_cable_i(:,index_set_j) + w_dAr;

                    end
                end
                cable_col_sensitivity_matrices{i} = sensitivity_matrix_cable_i;
            end

            % part 2: propeller actuation mapping (force and torque)
            % go through each propeller, generate the propeller velocity Jacobian along the given direction in local frame, then derive the propeller's corresponding row in T
            for i = 1:obj.numPropellers

                propeller_index = obj.propellers(i);
                install_link = obj.propeller_data_link_indices(propeller_index);
                tloc = obj.propeller_data_locations(:, propeller_index);
                fdir = -obj.propeller_data_directions(:, propeller_index); % force direction
                torque_coefficient = obj.propeller_data_c(propeller_index);

                wt = [torque_coefficient*fdir; fdir];
                JLt = obj.LinkJacobianLibrary(:,:,install_link);
                Jt = xltMMat(tloc)*JLt;
                
                % d/dx xltMMat([x;y;z]), d/dz xltMMat([x;y;z]) and d/dy xltMMat([x;y;z])
                tmp_dx = [ zeros(3)    	zeros(3);
                            -vecX3D([1;0;0])  zeros(3)];
                tmp_dy = [ zeros(3)    	zeros(3);
                            -vecX3D([0;1;0])  zeros(3)];
                tmp_dz = [ zeros(3)    	zeros(3);
                            -vecX3D([0;0;1])  zeros(3)];
                JtTwt_dtloc = [JLt'*tmp_dx'*wt,JLt'*tmp_dy'*wt,JLt'*tmp_dz'*wt]; %d/dtloc (Jt'wt)
                wt_dtdir = -[torque_coefficient*eye(3);eye(3)];

                % w_dtltd = [JtTwt_dtloc, Jt'*wt_dtdir];
                if thruster_base_reconfigurability(i)
                    thruster_col_base_sensitivity_matrices{i} = JtTwt_dtloc;
                end
                if thruster_direction_reconfigurability(i)
                    thruster_col_direction_sensitivity_matrices{i} = Jt'*wt_dtdir;
                end
            end

        end
        

            
        % function that gives out the constraint force (from joint's mechanical constraints)
        %   assume that the system is already updated prior to the calling of this function
        function [f_lib, f_constraint_lib] =  constraintWrenches(obj, u_cable, u_thruster, u_joint)

            % TODO: basically what needs to be done is to collect all the forces applied to each link and project the net force to the joint constraint directions
            % more details:     1) the link wrenches due to actuation (propellers and cables) need to be derived, this is easy for propellers but a little bit complicated for cable forces
            %                   2) the link wrenches required to maintain the velocity and acceleration should also be considered.
            %                   3) combine all the link wrenches and project to the null space of each link's Jacobian transpose (S_link(i)')
            
            
            % derive the external link wrenches caused by cable forces
            fx = zeros(6, obj.numLinks);

            % go through each cable
            for i = 1:obj.numCables
                attachment_index = obj.cables{i};
                numSegments = length(attachment_index) - 1;
                tmp_index = attachment_index(1);
                prev_link_index = obj.cable_data_attachment_link_indices(tmp_index);
                prev_attachment = obj.cable_data_attachments(:,tmp_index);
                for j = 1:numSegments

                    tmp_index = attachment_index(j+1);
                    new_link_index = obj.cable_data_attachment_link_indices(tmp_index);
                    new_attachment = obj.cable_data_attachments(:,tmp_index);

                    % wrench on the previous link
                    if prev_link_index > 0
                        new_attachment = obj.pointCoordinate(new_link_index, new_attachment, prev_link_index);
                        segment_ij_p = new_attachment - prev_attachment;
                        fx(:,prev_link_index) = fx(:,prev_link_index) + [vecX3D(prev_attachment-obj.CoM(:,prev_link_index)); eye(3)]*segment_ij_p*u_cable(i)/norm(segment_ij_p);
                    end

                    % wrench on the subsequent link
                    prev_attachment = obj.pointCoordinate(prev_link_index, prev_attachment, new_link_index);
                    segment_ij_s = prev_attachment - new_attachment;
                    fx(:,new_link_index) = fx(:,new_link_index) + [vecX3D(new_attachment-obj.CoM(:,new_link_index)); eye(3)]*segment_ij_s*u_cable(i)/norm(segment_ij_s);

                    prev_link_index = new_link_index;
                    prev_attachment = new_attachment;
                    
                end
            end

            % go through each thruster
            for i = 1:obj.numPropellers
                
                model_def.propeller_data_link_indices = [];
                model_def.propeller_data_directions = [];
                model_def.propeller_data_c = [];
                model_def.propeller_data_locations = [];

                install_link = obj.propeller_data_link_indices(i);
                install_point = obj.propeller_data_locations(:, i);
                force_direction = -obj.propeller_data_directions(:, i); % force direction
                torque_coefficient = obj.propeller_data_c(i);

                link_force_i = u_thruster(i)*[torque_coefficient*force_direction + vecX3D(install_point-obj.CoM(:,install_link))*force_direction; force_direction];

                fx(:,install_link) = fx(:,install_link) + link_force_i;

            end

            % go through each active joint (NOT 100% CERTAIN ABOUT THIS, BUGs POTENTIALLY EXIST)
            dof_counter = 0;
            active_dof_counter = 0;
            for i = 1:obj.numLinks

                joint_dof = obj.dimVelocityEachLink(i);
                
                if obj.joint_active(i)

                    f_active_joint = obj.S_links(:,counter+1:counter+joint_dof)*u_joint(active_dof_counter+1:active_dof_counter+joint_dof);

                    prev_link_index = obj.parent_link(i);
                    current_link_index = i;
                    if prev_link_index > 0
                        fx(:,prev_link_index) = fx(:,prev_link_index) - f_active_joint;
                    end
                    fx(:,current_link_index) = fx(:,current_link_index) + f_active_joint;

                    active_dof_counter = active_dof_counter + joint_dof;
                end

                dof_counter = dof_counter + joint_dof;

            end
            
            f_lib = zeros(6, obj.numLinks);
            acc_lib = zeros(6, obj.numLinks);
            f_constraint_lib = zeros(6, obj.numLinks);
            
            acc = [0; 0; 0; 0; 0; obj.g];
            

            for i = 1:obj.numLinks
                interval_dimVel_i = obj.dof_interval_links(1,i):obj.dof_interval_links(2,i);
                I_s_i           =   obj.I_s(:,:,i);
                vel             =   obj.vel_links(:,i);
                XM              =   obj.XM_links(:,:,i);
                p_i             =   obj.parent_link(i);
                if p_i ~= 0
                    % acc_lib(:,i)    =   XM*acc_lib(:,p_i) + obj.S_links(:,interval_dimVel_i)*obj.x_vel_dot(interval_dimVel_i) + obj.S_dot_links(:,interval_dimVel_i)*obj.x_vel(interval_dimVel_i);
                    acc_lib(:,i)    =   XM*acc_lib(:,p_i) + obj.S_dot_links(:,interval_dimVel_i)*obj.x_vel(interval_dimVel_i);
                else
                    % acc_lib(:,i)    =   XM*acc + obj.S_links(:,interval_dimVel_i)*obj.x_vel_dot(interval_dimVel_i) + obj.S_dot_links(:,interval_dimVel_i)*obj.x_vel(interval_dimVel_i);
                    acc_lib(:,i)    =   XM*acc + obj.S_dot_links(:,interval_dimVel_i)*obj.x_vel(interval_dimVel_i);
                end
                f_lib(:,i)      =   I_s_i*acc_lib(:,i) + vecX6DF(vel)*I_s_i*vel - fx(:,i);
            end
            
            for j = 1:obj.numLinks
                i           =   obj.numLinks + 1 - j;
                interval_dimVel_i = obj.dof_interval_links(1,i):obj.dof_interval_links(2,i);
                f_constraint_lib(:,i) = f_lib(:,i) - obj.S_links(:,interval_dimVel_i)*obj.S_links(:,interval_dimVel_i)'*f_lib(:,i);
                i_p = obj.parent_link(i);
                if i_p ~= 0
                    XF              =   obj.XF_links(:,:,i);
                    f_lib(:,i_p)    =   f_lib(:,i_p) + XF*f_lib(:,i);
                end
            end
            
            
        end
        
        % function that accepts a new system state and update the related
        % variables (there are two fake inputs that are not used. they
        % exists to align the update function call of this model with that
        % of CASPR model)
        function update(obj, pos, vel, acc, wext)

            obj.x_vel_dot = acc;
            obj.wext = wext;

            obj.updateSystemState(pos, vel);
            
            % update M, C, G, AM
            obj.updateM();
            % obj.updateC();
            % obj.updateG();
            obj.updateCG();
            obj.updateActuationMapping();
            
        end
        
        % given a system state, a control input and an external
        % disturbance (which shows up on the r.h.s. of the EoM), return the
        % state derivative.
        function x_dot = eom(obj, x, u, wext)
            pos = x(1:obj.dimPosition);
            vel = x(obj.dimPosition+1:obj.dimPosition+obj.dimVelocity);
            obj.update(pos, vel, zeros(size(vel)), zeros(size(vel)));
            acc = obj.M\([-obj.L',obj.A]*u-obj.C-obj.G + wext);
            
            x_dot = [obj.x_pos_dot; acc];
        end
        
        
        % function that accepts a state, an input, an external disturbance,
        % a time interval and a integration method name, return the
        % subsequent state
        function [pos_next, vel_next] = systemPropagation(obj, pos, vel, u, wext, dt, method)
            x0 = [pos; vel];
            if strcmp(method, 'euler')
                [~, x_out] = ode1(@(~,x) obj.eom(x, u, wext), [0 dt], x0);
                s_end = size(x_out, 1);
                pos_next = x_out(s_end, 1:obj.dimPosition)';
                vel_next = x_out(s_end, 1+obj.dimPosition:obj.dimPosition+obj.dimVelocity)';
            elseif strcmp(method, 'ode4')
                [~, x_out] = ode4(@(~,x) obj.eom(x, u, wext), [0 dt], x0);
                s_end = size(x_out, 1);
                pos_next = x_out(s_end, 1:obj.dimPosition)';
                vel_next = x_out(s_end, 1+obj.dimPosition:obj.dimPosition+obj.dimVelocity)';
            else
                % defult method is Euler
                x = [pos; vel];
                x_dot = obj.eom(x, u, wext);
                x = x + dt*x_dot;
                pos_next = x(1:obj.dimPosition);
                vel_next = x(1+obj.dimPosition:obj.dimPosition+obj.dimVelocity);
            end

            pos_next = obj.normalization(pos_next);
%             % normalize the new pos state (in case there exist quaternions)
%             x_pos_count =   0;
%             for i = 1:obj.numLinks
%                 joint_type_i            =   obj.joint_type_indices(i);
%                 dimPosition_i           =   obj.dimPositionEachLink(i);
%                 interval_pos            =   x_pos_count+1 : x_pos_count+dimPosition_i;
%                 pos_next(interval_pos)  =   joint_normalize(joint_type_i, pos_next(interval_pos));
%                 x_pos_count             =   x_pos_count + dimPosition_i;
%             end
        end
        
        function x_pos_normalized = normalization(obj, x_pos_ori)
            pose = x_pos_ori;
            % normalize the new pos state (in case there exist quaternions)
            x_pos_count =   0;
            for i = 1:obj.numLinks
                joint_type_i            =   obj.joint_type_indices(i);
                dimPosition_i           =   obj.dimPositionEachLink(i);
                interval_pos            =   x_pos_count+1 : x_pos_count+dimPosition_i;
                pose(interval_pos)      =   joint_normalize(joint_type_i, pose(interval_pos));
                x_pos_count             =   x_pos_count + dimPosition_i;
            end
            x_pos_normalized = pose;
        end
        
        % configuration update functions
        % function that updates a certain cable attachment location
        function updateCableAttachment(obj, cable_index, in_cable_attachment_index, new_attachment_coordinate)
            cable_attachment_index = obj.cables{cable_index}(in_cable_attachment_index);
            obj.cable_data_attachments(:, cable_attachment_index) = new_attachment_coordinate;
        end
        % function that updates a propeller's location and pointing direction
        function updatePropeller(obj, propeller_index, new_installing_location, new_pointing_direction)
            if ~isempty(new_installing_location)
                obj.propeller_data_locations(:, obj.propellers(propeller_index)) = new_installing_location;
            end
            if ~isempty(new_pointing_direction)
                obj.propeller_data_directions(:, obj.propellers(propeller_index)) = new_pointing_direction;
            end
        end
        % function that updates link's property and connecting point (so since this is a rigid body model, links won't deform hence the initial relative orientation of link attached frames won't change)
        function updateLink(obj, link_index, new_m, new_CoM, new_I_CoM, new_child_link_attaching_point, child_link_index)
            if ~isempty(new_m)
                obj.m(link_index)       =   new_m;
            end
            if ~isempty(new_CoM)
                obj.CoM(:, link_index)  =   new_CoM;
            end
            if ~isempty(new_I_CoM)
                obj.I_CoM{link_index}   =   new_I_CoM;
            end

            if ~isempty(new_child_link_attaching_point)
                % consider updating the child link attachment location
                if nargin >= 7
                    % use the given child link index
                else
                    % assume a most simple link topology is assumed to guess the child link index
                    child_link_index = link_index + 1;
                end
                if obj.parent_link(child_link_index) ~= link_index
                    error('The child link not found!');
                else
                    if child_link_index <= obj.numLinks && ~isempty(new_child_link_attaching_point)
                        obj.PL(:, child_link_index) =   new_child_link_attaching_point;
                    end
                end
            end

            obj.I_s(:,:,link_index) = spatialInertiaTensor(obj.I_CoM{link_index}, obj.CoM(:,link_index), obj.m(link_index));
        end
        
        function changeGravitationalAcceleration(obj, new_g)
            obj.g = new_g;
        end
        
        % getters
        function value = get.numDofs(obj)
            value = obj.dimVelocity;
        end
        function value = get.numDofVars(obj)
            value = obj.dimPosition;
        end
        function value = get.numActuatorsActive(obj)
            value = obj.numActuators;
        end
        function value = get.q_deriv(obj)
            value = obj.x_pos_dot;
        end
        function value = get.q_ddot_dynamics(obj)
            x_dot = obj.eom([obj.x_pos; obj.x_vel], obj.actuationForces, obj.wext);
            value = x_dot(obj.dimPosition+1:end);
        end
        function value = get.q(obj)
            value = obj.x_pos;
        end
        function value = get.q_dot(obj)
            value = obj.x_vel;
        end
        
    end
    
    methods
        
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % quaternion utility functions: see in folder ../quaternion_operations

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % spatial vector operations: see in folder ../spatial_vector_algebra
        
    end
end
