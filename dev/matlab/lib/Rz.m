function H = Rz(yaw)
% Rotation matrix about Z axis (yaw).

sy = sin(yaw);
cy = cos(yaw);

H = [cy, -sy, 0;
    sy, cy, 0;
    0, 0, 1];