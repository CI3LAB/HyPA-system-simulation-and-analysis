function n = AxisAngleToQuaternion(theta, axis)
    axis = axis/norm(axis);
    n = [cos(theta/2); sin(theta/2)*axis];
end