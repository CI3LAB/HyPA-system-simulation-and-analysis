% given a quaternion, compute the matrix E, which is used in a number of quaternion operations
% reference see
%   B. Graf, “Quaternions and dynamics,” Nov. 2008.
function E = QuaternionMatrixE(n)
    q0 = n(1);
    q1 = n(2);
    q2 = n(3);
    q3 = n(4);
    E = [-q1  q0 -q3  q2;
         -q2  q3  q0 -q1
         -q3 -q2  q1  q0];
end