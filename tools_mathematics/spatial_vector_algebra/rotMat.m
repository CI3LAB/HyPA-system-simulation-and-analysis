% function that takes a rotation matrix from frame A to B and gives back a
% spatial vector transformation from the corresponding 6-D space A to B
% that is a direct result of the rotation represented by E
function res = rotMat(E)
    % res = blkdiag(E,E); % this seems to be slower than the implementation below
    res = zeros(6, 6);
    res(1:3, 1:3) = E;
    res(4:6, 4:6) = E;
end