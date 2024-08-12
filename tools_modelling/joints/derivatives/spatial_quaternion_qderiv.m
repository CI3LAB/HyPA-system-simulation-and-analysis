% function that computes the derivative of q
%   input: q = [pos; n] where n is the quaternion and pos is the position coordinate in the PARENT FRAME
%          v = [pos_dot; omega] where omega is the angular velocity defined in the LOCAL FRAME
%                               and pos_dot is the linear velocity defined in the PARENT FRAME
function q_derivative = spatial_quaternion_qderiv(q, v)
    % initialize
    q_derivative = zeros(length(q),1);
    % get the linear velocity
    q_derivative(1:3) = v(1:3);
    % get the quaternion dot
    q_derivative(4:7) = QuaternionDerivative(q(4:7), v(4:6));
end