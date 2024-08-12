model_def.robotName = 'APAS_var0'; % adopts large (realistic) scale (size and mass)
        % robot name, a string

%% body definition
pyramid_init_height_scalar = 2.5;
module_platform_dist_scalar = 5;
module_size_scalar = 6;
platform_size_scalar = 6;
accessory_size_scalar = 1.5;
module_weight_scalar = 100;
platform_weight_scalar = 60;
accessary_weight_scalar = 10;
num_links = 7;
% there are 7 links:
model_def.link_name  = {'crane_hook',...
                        'steering_control_box',...
                        'steering_shaft',...
                        'hoisting_platform',...
                        'horizontal_moving_platform',...
                        'vertical_moving_platform',...
                        'module'...
                        };
% link 0: attached to the crane trolley, which is fixed in the ground frame
% ground frame (frame 0) is fixed on the tower crane base
% % internal def variable
left_right_cable_dist = accessory_size_scalar * 0.58;
height_crane_trolley = 60.0;
crane_cable_length = 46.0;
% height_crane_trolley = 6.0;
% crane_cable_length = 1.8;
trolley_x_shift = 5.0;
trolley_y_shift = 5.0;
crane_hook_init_position = [trolley_x_shift; trolley_y_shift; height_crane_trolley-crane_cable_length];
% % subsequent link definition
crane_hook_joint_coord = crane_hook_init_position;
% % cable attachment point definition
left_cable_hook_on_trolley = [-left_right_cable_dist/2.0 + trolley_x_shift; trolley_y_shift; height_crane_trolley];
right_cable_hook_on_trolley = [left_right_cable_dist/2.0 + trolley_x_shift; trolley_y_shift; height_crane_trolley];
trolley_cable_attachment_set = [left_cable_hook_on_trolley, right_cable_hook_on_trolley];

% link 1: crane hook + lifting gear base
% frame defined at the top of the crane hook block
% crane cable attached on top of the crane hook block
% % internal def variable
hook_length = accessory_size_scalar * 0.08;
thickness_crane_hook = accessory_size_scalar * 0.13;
height_crane_hook = accessory_size_scalar * 0.32;
width_crane_hook = accessory_size_scalar * 0.8;
side_height_crane_hook = accessory_size_scalar * 0.075;
bottom_width_crane_hook = accessory_size_scalar * 0.16;
y1_pos_crane_hook = thickness_crane_hook/2.0;
y1_neg_crane_hook = -thickness_crane_hook/2.0;
y2_pos_crane_hook = thickness_crane_hook/2.0;
y2_neg_crane_hook = -thickness_crane_hook/2.0;
y3_pos_crane_hook = thickness_crane_hook/2.0;
y3_neg_crane_hook = -thickness_crane_hook/2.0;
x1_pos_crane_hook = width_crane_hook/2.0;
x1_neg_crane_hook = -width_crane_hook/2.0;
x2_pos_crane_hook = width_crane_hook/2.0;
x2_neg_crane_hook = -width_crane_hook/2.0;
x3_pos_crane_hook = bottom_width_crane_hook/2.0;
x3_neg_crane_hook = -bottom_width_crane_hook/2.0;
z1_crane_hook = 0.0;
z2_crane_hook = -side_height_crane_hook;
z3_crane_hook = -height_crane_hook;
% % mass and inertia definition
mass_crane_hook = accessary_weight_scalar * 10;
mass_lifting_gear_base = accessary_weight_scalar * 5;
% lX_lifting_gear = 0.45;
% lY_lifting_gear = 0.17;
% lZ_lifting_gear = 0.16;
% l_Link = 0.16;
I_crane_hook = accessary_weight_scalar * accessory_size_scalar^2 * diag([0.149, 0.5673, 0.6558]);
CoM_crane_hook = accessory_size_scalar * [0.0; 0.0; -0.114];
% % link geometry definition
crane_hook_box_geometry = [ x1_pos_crane_hook, y1_pos_crane_hook, z1_crane_hook;
                            x1_pos_crane_hook, y1_neg_crane_hook, z1_crane_hook;
                            x1_neg_crane_hook, y1_pos_crane_hook, z1_crane_hook;
                            x1_neg_crane_hook, y1_neg_crane_hook, z1_crane_hook; % top level
                            x2_pos_crane_hook, y2_pos_crane_hook, z2_crane_hook;
                            x2_pos_crane_hook, y2_neg_crane_hook, z2_crane_hook;
                            x2_neg_crane_hook, y2_pos_crane_hook, z2_crane_hook;
                            x2_neg_crane_hook, y2_neg_crane_hook, z2_crane_hook; % mid level
                            x3_pos_crane_hook, y3_pos_crane_hook, z3_crane_hook;
                            x3_pos_crane_hook, y3_neg_crane_hook, z3_crane_hook;
                            x3_neg_crane_hook, y3_pos_crane_hook, z3_crane_hook;
                            x3_neg_crane_hook, y3_neg_crane_hook, z3_crane_hook; % bottom level
                            ];
hook_geometry = [   0.0, 0.0, -height_crane_hook;
                    0.0, 0.0, -height_crane_hook-hook_length];
cable_hook_vertices = {crane_hook_box_geometry, hook_geometry};
% % subsequent link definition
hook_universal_joint_coord = [0.0; 0.0; -height_crane_hook-hook_length];
% % cable attachment point definition
left_cable_hook = [-left_right_cable_dist/2.0; 0.0; 0.0];
right_cable_hook = [left_right_cable_dist/2.0; 0.0; 0.0];
crane_hook_cable_attachment_set = [left_cable_hook, right_cable_hook];

% link 2: steering control box
% % internal def variable
length_shaft = 0.1;
steering_control_box_height1 = accessory_size_scalar * 0.1;
steering_control_box_height2 = accessory_size_scalar * 0.03;
steering_control_box_bottom_square_size = accessory_size_scalar * 0.06;
steering_control_box_length_x = accessory_size_scalar * 0.45;
steering_control_box_length_y = accessory_size_scalar * 0.17;
y1_pos_steering_box = steering_control_box_length_y/2.0;
y1_neg_steering_box = -steering_control_box_length_y/2.0;
y2_pos_steering_box = steering_control_box_length_y/2.0;
y2_neg_steering_box = -steering_control_box_length_y/2.0;
y3_pos_steering_box = steering_control_box_bottom_square_size/2.0;
y3_neg_steering_box = -steering_control_box_bottom_square_size/2.0;
x1_pos_steering_box = steering_control_box_length_x/2.0;
x1_neg_steering_box = -steering_control_box_length_x/2.0;
x2_pos_steering_box = steering_control_box_length_x/2.0;
x2_neg_steering_box = -steering_control_box_length_x/2.0;
x3_pos_steering_box = steering_control_box_bottom_square_size/2.0;
x3_neg_steering_box = -steering_control_box_bottom_square_size/2.0;
z1_steering_box = -hook_length;
z2_steering_box = -hook_length-steering_control_box_height1;
z3_steering_box = -hook_length-steering_control_box_height1-steering_control_box_height2;
% % mass and inertia definition
mass_steering_control_box = accessary_weight_scalar * 5;
I_steering_control_box = accessary_weight_scalar * accessory_size_scalar^2 * diag([0.02, 0.1, 0.1]);
CoM_steering_control_box = [0.0; 0.0; -hook_length];
% % link geometry definition
steering_control_box_geometry = [   x1_pos_steering_box, y1_pos_steering_box, z1_steering_box;
                                    x1_pos_steering_box, y1_neg_steering_box, z1_steering_box;
                                    x1_neg_steering_box, y1_pos_steering_box, z1_steering_box;
                                    x1_neg_steering_box, y1_neg_steering_box, z1_steering_box; % top level
                                    x2_pos_steering_box, y2_pos_steering_box, z2_steering_box;
                                    x2_pos_steering_box, y2_neg_steering_box, z2_steering_box;
                                    x2_neg_steering_box, y2_pos_steering_box, z2_steering_box;
                                    x2_neg_steering_box, y2_neg_steering_box, z2_steering_box; % mid level
                                    x3_pos_steering_box, y3_pos_steering_box, z3_steering_box;
                                    x3_pos_steering_box, y3_neg_steering_box, z3_steering_box;
                                    x3_neg_steering_box, y3_pos_steering_box, z3_steering_box;
                                    x3_neg_steering_box, y3_neg_steering_box, z3_steering_box; % bottom level
                                    ];
hook_geometry = [   0.0, 0.0, 0.0;
                    0.0, 0.0, -hook_length];
steering_control_box_vertices = {steering_control_box_geometry, hook_geometry};
% % subsequent link definition
steering_shaft_revolute_joint_coord = [0.0; 0.0; z3_steering_box];
% % cable attachment point definition


% link 3: lifting gear's rotating shaft
% % internal def variable
length_shaft = accessory_size_scalar * 0.1;
hook_block_height = accessory_size_scalar * 0.1;
hook_block_size = accessory_size_scalar * 0.2;
x_pos_hook_block = hook_block_size/2.0;
x_neg_hook_block = -hook_block_size/2.0;
y_pos_hook_block = hook_block_size/2.0;
y_neg_hook_block = -hook_block_size/2.0;
z_pos_hook_block = 0.0;
z_neg_hook_block = -hook_block_height;
initial_pyramid_height = pyramid_init_height_scalar * 2.0;
% % mass and inertia definition
mass_lifting_gear_shaft = accessary_weight_scalar * 0.6;
I_shaft = accessary_weight_scalar * accessory_size_scalar^2 * diag([0.006, 0.006, 0.01]);
CoM_shaft = [0.0; 0.0; -hook_block_height/2.0];
% % link geometry definition
hook_block_geometry = [ x_pos_hook_block, y_pos_hook_block, z_pos_hook_block;
                        x_pos_hook_block, y_pos_hook_block, z_neg_hook_block;
                        x_pos_hook_block, y_neg_hook_block, z_pos_hook_block;
                        x_pos_hook_block, y_neg_hook_block, z_neg_hook_block;
                        x_neg_hook_block, y_pos_hook_block, z_pos_hook_block;
                        x_neg_hook_block, y_pos_hook_block, z_neg_hook_block;
                        x_neg_hook_block, y_neg_hook_block, z_pos_hook_block;
                        x_neg_hook_block, y_neg_hook_block, z_neg_hook_block;
                        ];
shaft_geometry = [  0.0, 0.0, length_shaft;
                    0.0, 0.0, 0.0];
shaft_vertices = {hook_block_geometry, shaft_geometry};
% % subsequent link definition
hoisting_frame_spatial_joint_coord = [0.0; 0.0; z_neg_hook_block - initial_pyramid_height];
% % cable attachment point definition
hoisting_cable_attachment_offset_x = accessory_size_scalar * 0.05;
hoisting_cable_attachment_offset_y = accessory_size_scalar * 0.05;
cable_attachment_rectangle_x = 1.0*hoisting_cable_attachment_offset_x;
cable_attachment_rectangle_y = 1.0*hoisting_cable_attachment_offset_y;
x_neg_shaft_cable_attachment = -cable_attachment_rectangle_x;
x_pos_shaft_cable_attachment = cable_attachment_rectangle_x;
y_neg_shaft_cable_attachment = -cable_attachment_rectangle_y;
y_pos_shaft_cable_attachment = cable_attachment_rectangle_y;
attachment_xneg_yneg = [x_neg_shaft_cable_attachment; y_neg_shaft_cable_attachment; z_neg_hook_block];
attachment_xpos_yneg = [x_pos_shaft_cable_attachment; y_neg_shaft_cable_attachment; z_neg_hook_block];
attachment_xpos_ypos = [x_pos_shaft_cable_attachment; y_pos_shaft_cable_attachment; z_neg_hook_block];
attachment_xneg_ypos = [x_neg_shaft_cable_attachment; y_pos_shaft_cable_attachment; z_neg_hook_block];
lifting_gear_shaft_cable_attachment_set = [attachment_xneg_yneg, attachment_xpos_yneg, attachment_xpos_ypos, attachment_xneg_ypos];
% shaft_end_point = [0.0; 0.0; -length_shaft];
% lifting_gear_shaft_cable_attachment_set = [shaft_end_point, shaft_end_point, shaft_end_point, shaft_end_point];

% link 4: the hoisting frame
% % internal def variable
horizontal_platform_dist = platform_size_scalar * 0.08;
hoisting_frame_height = platform_size_scalar * 0.08;
hoisting_frame_length_x = platform_size_scalar * 1.3;
hoisting_frame_length_y = platform_size_scalar * 0.75;
x_pos_hoisting_frame = hoisting_frame_length_x/2.0;
x_neg_hoisting_frame = -hoisting_frame_length_x/2.0;
y_pos_hoisting_frame = hoisting_frame_length_y/2.0;
y_neg_hoisting_frame = -hoisting_frame_length_y/2.0;
z_pos_hoisting_frame = hoisting_frame_height/2.0;
z_neg_hoisting_frame = -hoisting_frame_height/2.0;
% % mass and inertia definition
mass_hoisting_frame = platform_weight_scalar * 50;
I_hoisting_frame = platform_weight_scalar * platform_size_scalar^2 * diag([3.0, 10.0, 12.5]);
CoM_hoisting_frame = [0.0; 0.0; 0.0];
% % link geometry definition
hoisting_frame_vertices = {
                          [ x_pos_hoisting_frame, y_pos_hoisting_frame, z_pos_hoisting_frame;
                            x_pos_hoisting_frame, y_pos_hoisting_frame, z_neg_hoisting_frame;
                            x_pos_hoisting_frame, y_neg_hoisting_frame, z_pos_hoisting_frame;
                            x_pos_hoisting_frame, y_neg_hoisting_frame, z_neg_hoisting_frame;
                            x_neg_hoisting_frame, y_pos_hoisting_frame, z_pos_hoisting_frame;
                            x_neg_hoisting_frame, y_pos_hoisting_frame, z_neg_hoisting_frame;
                            x_neg_hoisting_frame, y_neg_hoisting_frame, z_pos_hoisting_frame;
                            x_neg_hoisting_frame, y_neg_hoisting_frame, z_neg_hoisting_frame;
                            ]
                            };
% % subsequent link definition
horizontal_platform_X_prismatic_joint_coord = [0.0; 0.0; -horizontal_platform_dist];
% % cable attachment point definition
attachment_xneg_yneg = [x_neg_hoisting_frame; y_neg_hoisting_frame; z_pos_hoisting_frame];
attachment_xpos_yneg = [x_pos_hoisting_frame; y_neg_hoisting_frame; z_pos_hoisting_frame];
attachment_xpos_ypos = [x_pos_hoisting_frame; y_pos_hoisting_frame; z_pos_hoisting_frame];
attachment_xneg_ypos = [x_neg_hoisting_frame; y_pos_hoisting_frame; z_pos_hoisting_frame];
hoisting_frame_cable_attachment_set = [attachment_xneg_yneg, attachment_xpos_yneg, attachment_xpos_ypos, attachment_xneg_ypos];

cable_attachment_rectangle_x = 5*hoisting_cable_attachment_offset_x;
cable_attachment_rectangle_y = 5*hoisting_cable_attachment_offset_y;
cable_attachment_rectangle_x = platform_size_scalar * 0.05;
cable_attachment_rectangle_y = platform_size_scalar * 0.1;
x_neg_shaft_cable_attachment = -cable_attachment_rectangle_x;
x_pos_shaft_cable_attachment = cable_attachment_rectangle_x;
y_neg_shaft_cable_attachment = -cable_attachment_rectangle_y;
y_pos_shaft_cable_attachment = cable_attachment_rectangle_y;
attachment_xneg_yneg = [x_neg_shaft_cable_attachment; y_neg_shaft_cable_attachment; z_pos_hoisting_frame];
attachment_xpos_yneg = [x_pos_shaft_cable_attachment; y_neg_shaft_cable_attachment; z_pos_hoisting_frame];
attachment_xpos_ypos = [x_pos_shaft_cable_attachment; y_pos_shaft_cable_attachment; z_pos_hoisting_frame];
attachment_xneg_ypos = [x_neg_shaft_cable_attachment; y_pos_shaft_cable_attachment; z_pos_hoisting_frame];
hoisting_frame_cable_attachment_set2 = [attachment_xneg_yneg, attachment_xpos_yneg, attachment_xpos_ypos, attachment_xneg_ypos];


% link 5: the horizontal motion platform
% % internal def variable
vertical_platform_dist = platform_size_scalar * 0.08;
horizontal_platform_height = platform_size_scalar * 0.06;
horizontal_platform_length_x = platform_size_scalar * 0.5;
horizontal_platform_length_y = platform_size_scalar * 0.784;
x_pos_horizontal_platform = horizontal_platform_length_x/2.0;
x_neg_horizontal_platform = -horizontal_platform_length_x/2.0;
y_pos_horizontal_platform = horizontal_platform_length_y/2.0;
y_neg_horizontal_platform = -horizontal_platform_length_y/2.0;
z_pos_horizontal_platform = horizontal_platform_height/2.0;
z_neg_horizontal_platform = -horizontal_platform_height/2.0;
% % mass and inertia definition
mass_horizontal_platform = platform_weight_scalar * 10;
I_horizontal_platform = platform_weight_scalar * platform_size_scalar^2 * diag([0.85, 0.4, 1.127]);
CoM_horizontal_platform = platform_size_scalar * [0.0; 0.0; 0.0];
% % link geometry definition
horizontal_platform_vertices = {
                               [x_pos_horizontal_platform, y_pos_horizontal_platform, z_pos_horizontal_platform;
                                x_pos_horizontal_platform, y_pos_horizontal_platform, z_neg_horizontal_platform;
                                x_pos_horizontal_platform, y_neg_horizontal_platform, z_pos_horizontal_platform;
                                x_pos_horizontal_platform, y_neg_horizontal_platform, z_neg_horizontal_platform;
                                x_neg_horizontal_platform, y_pos_horizontal_platform, z_pos_horizontal_platform;
                                x_neg_horizontal_platform, y_pos_horizontal_platform, z_neg_horizontal_platform;
                                x_neg_horizontal_platform, y_neg_horizontal_platform, z_pos_horizontal_platform;
                                x_neg_horizontal_platform, y_neg_horizontal_platform, z_neg_horizontal_platform;
                                ]
                                };
% % subsequent link definition
vertical_platform_Y_prismatic_joint_coord = [0.0; 0.0; -vertical_platform_dist];
% % cable attachment point definition


% link 7: the module
% % internal def variable
module_height = module_size_scalar * 0.5;
module_length_x = module_size_scalar * 1.02;
module_length_y = module_size_scalar * 0.52;
x_pos_module = module_length_x/2.0;
x_neg_module = -module_length_x/2.0;
y_pos_module = module_length_y/2.0;
y_neg_module = -module_length_y/2.0;
z_pos_module = module_height/2.0;
z_neg_module = -module_height/2.0;
% % mass and inertia definition
mass_module = module_weight_scalar * 200;
I_module = module_weight_scalar * module_size_scalar^2 * diag([9.03, 22.39, 22.74]);
CoM_module = module_size_scalar * [0.0; 0.0; 0.0];
% % link geometry definition
module_vertices = {
                  [ x_pos_module, y_pos_module, z_pos_module;
                    x_pos_module, y_pos_module, z_neg_module;
                    x_pos_module, y_neg_module, z_pos_module;
                    x_pos_module, y_neg_module, z_neg_module;
                    x_neg_module, y_pos_module, z_pos_module;
                    x_neg_module, y_pos_module, z_neg_module;
                    x_neg_module, y_neg_module, z_pos_module;
                    x_neg_module, y_neg_module, z_neg_module;
                    ]
                    };
% % subsequent link definition
% % cable attachment point definition
attachment_xneg_yneg = [x_neg_module; y_neg_module; z_pos_module];
attachment_xpos_yneg = [x_pos_module; y_neg_module; z_pos_module];
attachment_xpos_ypos = [x_pos_module; y_pos_module; z_pos_module];
attachment_xneg_ypos = [x_neg_module; y_pos_module; z_pos_module];
module_cable_attachment_set = [attachment_xneg_yneg, attachment_xpos_yneg, attachment_xpos_ypos, attachment_xneg_ypos];



% link 6: the vertical motion platform
% % internal def variable
module_dist = module_platform_dist_scalar * 0.8;
vertical_platform_height = platform_size_scalar * 0.06;
vertical_platform_length_x = platform_size_scalar * 1.1;
vertical_platform_length_y = platform_size_scalar * 0.634;
x_pos_vertical_platform = vertical_platform_length_x/2.0;
x_neg_vertical_platform = -vertical_platform_length_x/2.0;
y_pos_vertical_platform = vertical_platform_length_y/2.0;
y_neg_vertical_platform = -vertical_platform_length_y/2.0;
z_pos_vertical_platform = vertical_platform_height/2.0;
z_neg_vertical_platform = -vertical_platform_height/2.0;
% % mass and inertia definition
mass_vertical_platform = platform_weight_scalar * 10;
I_vertical_platform = platform_weight_scalar * platform_size_scalar^2 * diag([0.24, 1.32, 1.466]);
CoM_vertical_platform = platform_size_scalar * [0.0; 0.0; 0.0];
% % link geometry definition
vertical_platform_vertices = {
                             [  x_pos_vertical_platform, y_pos_vertical_platform, z_pos_vertical_platform;
                                x_pos_vertical_platform, y_pos_vertical_platform, z_neg_vertical_platform;
                                x_pos_vertical_platform, y_neg_vertical_platform, z_pos_vertical_platform;
                                x_pos_vertical_platform, y_neg_vertical_platform, z_neg_vertical_platform;
                                x_neg_vertical_platform, y_pos_vertical_platform, z_pos_vertical_platform;
                                x_neg_vertical_platform, y_pos_vertical_platform, z_neg_vertical_platform;
                                x_neg_vertical_platform, y_neg_vertical_platform, z_pos_vertical_platform;
                                x_neg_vertical_platform, y_neg_vertical_platform, z_neg_vertical_platform;
                                ]
                                };
% % subsequent link definition
module_spatial_joint_coord = [0.0; 0.0; -module_dist];
% % cable attachment point definition
attachment_xneg_yneg = [x_neg_module; y_neg_module; z_neg_vertical_platform];
attachment_xpos_yneg = [x_pos_module; y_neg_module; z_neg_vertical_platform];
attachment_xpos_ypos = [x_pos_module; y_pos_module; z_neg_vertical_platform];
attachment_xneg_ypos = [x_neg_module; y_pos_module; z_neg_vertical_platform];
vertical_platform_cable_attachment_set = [attachment_xneg_yneg, attachment_xpos_yneg, attachment_xpos_ypos, attachment_xneg_ypos];


model_def.CoM = [   CoM_crane_hook,...
                    CoM_steering_control_box,...
                    CoM_shaft,...
                    CoM_hoisting_frame,...
                    CoM_horizontal_platform,...
                    CoM_vertical_platform,...
                    CoM_module];     
        % each link's center of mass coordinate in local frame, a 2D array with the second dimension being the link index
model_def.m = [ mass_crane_hook + mass_lifting_gear_base, ...
                mass_steering_control_box, ...
                mass_lifting_gear_shaft, ...
                mass_hoisting_frame, ...
                mass_horizontal_platform, ...
                mass_vertical_platform, ...
                mass_module];  
        % each link's mass, a row vector
model_def.I_CoM = {I_crane_hook, I_steering_control_box, I_shaft, I_hoisting_frame, I_horizontal_platform, I_vertical_platform, I_module};
        % each link's inertia tensors w.r.t. the corresponding CoM, a 1D cell array
model_def.PL = [crane_hook_joint_coord, hook_universal_joint_coord, steering_shaft_revolute_joint_coord, hoisting_frame_spatial_joint_coord, horizontal_platform_X_prismatic_joint_coord, vertical_platform_Y_prismatic_joint_coord, module_spatial_joint_coord];
        % each joint's coordinate in the parent link frame, a 2D array with the second dimension being the link index
model_def.VL = {cable_hook_vertices, steering_control_box_vertices, shaft_vertices, hoisting_frame_vertices, horizontal_platform_vertices, vertical_platform_vertices, module_vertices};
        % the link vertices in the current link frame, a cell array with each cell containing the vertices of a link component geometry and the combined geometry represents the link
model_def.joint_type = {'SpatialEulerXYZ', 'UniversalXY', 'RevoluteZ', 'SpatialEulerXYZ', 'LinearX', 'LinearY', 'SpatialEulerXYZ'};
        % the joint types for all the joints, a cell array
model_def.parent_link = [0, 1, 2, 3, 4, 5, 6];
        % the indices of parent links (0 represents the base)

%% active joint definition
model_def.joint_active = [0, 0, 1, 0, 1, 1, 0];
        % indicating whether a joint is active or passive, a 1D array
model_def.activeJointForcesMin = [-10; -500; -500];
model_def.activeJointForcesMax = [10; 500; 500];

%% cable definition
model_def.cables = {    [1,3],...   (constraint) crane cable 1
                        [2,4],...   (constraint) crane cable 2
                        [13, 5, 9],...  (control) hoisting frame cable 1
                        [14, 6, 10],... (control) hoisting frame cable 2
                        [15, 7, 11],... (control) hoisting frame cable 3
                        [16, 8, 12],... (control) hoisting frame cable 4
                        [17, 21],...(constraint) module cable 1
                        [18, 22],...(constraint) module cable 2
                        [19, 23],...(constraint) module cable 3
                        [20, 24] ...(constraint) module cable 4
                        };
        % the data entry indices of the attachments each cable has in the data files, a 1D cell array, attachments with small link index come first
model_def.cable_data_attachments = [    trolley_cable_attachment_set,...
                                        crane_hook_cable_attachment_set,...
                                        lifting_gear_shaft_cable_attachment_set,...
                                        hoisting_frame_cable_attachment_set,...
                                        hoisting_frame_cable_attachment_set2,...
                                        vertical_platform_cable_attachment_set,...
                                        module_cable_attachment_set...
                                        ];
        % each cable's attachment locations in link frame, a 2D array with the second dimension being the attachment index
model_def.cable_data_attachment_link_indices = [    0, 0,... 
                                                    1, 1,...
                                                    3, 3, 3, 3,...
                                                    4, 4, 4, 4,...
                                                    4, 4, 4, 4,...
                                                    6, 6, 6, 6,...
                                                    7, 7, 7, 7 ...
                                                    ];
        % each cable attachment's attached frame, a row vector
constraint_cable_force_limit = 1e7;
control_cable_force_limit = 1e4;
model_def.cableForcesMin = [0; 0; 0; 0; 0; 0; 0; 0; 0; 0];
model_def.cableForcesMax = [    constraint_cable_force_limit;...
                                constraint_cable_force_limit;...
                                control_cable_force_limit;...
                                control_cable_force_limit;...
                                control_cable_force_limit;...
                                control_cable_force_limit;...
                                constraint_cable_force_limit;...
                                constraint_cable_force_limit;...
                                constraint_cable_force_limit;...
                                constraint_cable_force_limit ...
                                ];

%% propeller definition
n = 8;
T_p = zeros(3,n);
T_d = zeros(3,n);
    
x_pos_dir = [1; 0; 0];
x_neg_dir = [-1; 0; 0];
y_pos_dir = [0; 1; 0];
y_neg_dir = [0; -1; 0];

const_dist = 0.05;



T_p(:,1) = [x_neg_hoisting_frame + 0;...
            y_neg_hoisting_frame + const_dist;...
            z_pos_hoisting_frame];
T_d(:,1) = x_neg_dir;
T_p(:,2) = [x_neg_hoisting_frame + const_dist;...
            y_neg_hoisting_frame + 0;...
            z_pos_hoisting_frame];
T_d(:,2) = y_neg_dir;

T_p(:,3) = [x_pos_hoisting_frame - const_dist;...
            y_neg_hoisting_frame + 0;...
            z_pos_hoisting_frame];
T_d(:,3) = y_neg_dir;
T_p(:,4) = [x_pos_hoisting_frame - 0;...
            y_neg_hoisting_frame + const_dist;...
            z_pos_hoisting_frame];
T_d(:,4) = x_pos_dir;

T_p(:,5) = [x_pos_hoisting_frame - 0;...
            y_pos_hoisting_frame - const_dist;...
            z_pos_hoisting_frame];
T_d(:,5) = x_pos_dir;
T_p(:,6) = [x_pos_hoisting_frame - const_dist;...
            y_pos_hoisting_frame - 0;...
            z_pos_hoisting_frame];
T_d(:,6) = y_pos_dir;

T_p(:,7) = [x_neg_hoisting_frame + const_dist;...
            y_pos_hoisting_frame - 0;...
            z_pos_hoisting_frame];
T_d(:,7) = y_pos_dir;
T_p(:,8) = [x_neg_hoisting_frame + 0;...
            y_pos_hoisting_frame - const_dist;...
            z_pos_hoisting_frame];
T_d(:,8) = x_neg_dir;

c = 0.05;

t_min = 0.0;
t_max = 80.0;

model_def.propellers = [1; 2; 3; 4; 5; 6; 7; 8];
        % the data entry indices of the propellers in the data files, a 1D array
model_def.propeller_data_link_indices = [4; 4; 4; 4; 4; 4; 4; 4];
        % the indices of locations propeller's attached frame
model_def.propeller_data_directions = T_d;
        % each propeller's pointing direction in link frame, a 2D array with the second dimension being the propeller index
model_def.propeller_data_c = [-c; c; -c; -c; c; -c; c; c];
        % each propeller's momentum-lift coefficient, a 1D array
model_def.propeller_data_locations = T_p;
        % each propeller's install position in link frame, a 2D array with the second dimension being the propeller index
model_def.thrusterForcesMin = [t_min; t_min; t_min; t_min; t_min; t_min; t_min; t_min];
model_def.thrusterForcesMax = [t_max; t_max; t_max; t_max; t_max; t_max; t_max; t_max];