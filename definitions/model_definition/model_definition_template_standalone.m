model_def.robotName = [];
        % robot name, a string

%% body definition
model_def.CoM = [];     
        % each link's center of mass coordinate in local frame, a 2D array with the second dimension being the link index
model_def.m = [];
        % each link's mass, a row vector
model_def.I_CoM = [];
        % each link's inertia tensors w.r.t. the corresponding CoM, a 1D cell array
model_def.PL = [];
        % each joint's coordinate in the parent link frame, a 2D array with the second dimension being the link index
model_def.VL = [];
        % the link vertices in the current link frame, a cell array with each cell containing the vertices of a link component geometry and the combined geometry represents the link
model_def.joint_type = [];
        % the joint types for all the joints, a cell array
model_def.parent_link = [];
        % the indices of parent links (0 represents the base)
model_def.link_name = [];
        % the name string of links, a cell array

%% active joint definition
model_def.joint_active = [];
        % indicating whether a joint is active or passive, a 1D array
model_def.activeJointForcesMin = [];
model_def.activeJointForcesMax = [];

%% cable definition
model_def.cables = [];
        % the data entry indices of the attachments each cable has in the data files, a 1D cell array, attachments with small link index come first
model_def.cable_data_attachments = [];
        % each cable's attachment locations in link frame, a 2D array with the second dimension being the attachment index
model_def.cable_data_attachment_link_indices = [];
        % each cable attachment's attached frame, a row vector
model_def.cableForcesMin = [];
model_def.cableForcesMax = [];

%% propeller definition
model_def.propellers = [];
        % the data entry indices of the propellers in the data files, a 1D array
model_def.propeller_data_link_indices = [];
        % the indices of locations propeller's attached frame
model_def.propeller_data_directions = [];
        % each propeller's pointing direction in link frame, a 2D array with the second dimension being the propeller index
model_def.propeller_data_c = [];
        % each propeller's momentum-lift coefficient, a 1D array
model_def.propeller_data_locations = [];
        % each propeller's install position in link frame, a 2D array with the second dimension being the propeller index
model_def.thrusterForcesMin = [];
model_def.thrusterForcesMax = [];