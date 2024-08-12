% a function that converts the string joint type into an internal index
% the joint type names include:
%       1. 'SpatialEulerXYZ'
%       2. 'SpatialQuaternion'
%       3. 'SphericalEulerXYZ'
%       4. 'SphericalQuaternion'
%       5. 'UniversalXY'


function joint_type_index = joint_type_interpretation(joint_type_string)
    if strcmp(joint_type_string, 'SpatialEulerXYZ')
        joint_type_index = 2;
    elseif strcmp(joint_type_string, 'SpatialQuaternion')
        joint_type_index = 1;
    elseif strcmp(joint_type_string, 'SphericalEulerXYZ')
        joint_type_index = 4;
    elseif strcmp(joint_type_string, 'SphericalQuaternion')
        joint_type_index = 3;
    elseif strcmp(joint_type_string, 'UniversalXY')
        joint_type_index = 5;
    elseif strcmp(joint_type_string, 'RevoluteX')
        joint_type_index = 6;
    elseif strcmp(joint_type_string, 'RevoluteY')
        joint_type_index = 7;
    elseif strcmp(joint_type_string, 'RevoluteZ')
        joint_type_index = 8;
    elseif strcmp(joint_type_string, 'UniversalXZ')
        joint_type_index = 9;
    elseif strcmp(joint_type_string, 'PlanarXZ')
        joint_type_index = 10;
    elseif strcmp(joint_type_string, 'PlanarXY')
        joint_type_index = 11;
    elseif strcmp(joint_type_string, 'PlanarYZ')
        joint_type_index = 12;
    elseif strcmp(joint_type_string, 'SpatialEulerZYX')
        joint_type_index = 13;
    elseif strcmp(joint_type_string, 'SphericalEulerZYX')
        joint_type_index = 14;
    elseif strcmp(joint_type_string, 'LinearX')
        joint_type_index = 15;
    elseif strcmp(joint_type_string, 'LinearY')
        joint_type_index = 16;
    elseif strcmp(joint_type_string, 'LinearZ')
        joint_type_index = 17;
    elseif strcmp(joint_type_string, 'LinearXY')
        joint_type_index = 18;
    elseif strcmp(joint_type_string, 'LinearXZ')
        joint_type_index = 19;
    elseif strcmp(joint_type_string, 'LinearYZ')
        joint_type_index = 20;
    else
        error('The ''%s'' joint type is not recognized.', joint_type_string);
    end
end