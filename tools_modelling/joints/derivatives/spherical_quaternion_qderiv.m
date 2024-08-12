% function that computes the derivative of q
%   input: q = [n] where n is the quaternion
%          v = [omega] where omega is the angular velocity defined in the LOCAL FRAME

function q_derivative = spherical_quaternion_qderiv(q, v)
    q_derivative = QuaternionDerivative(q, v);
end