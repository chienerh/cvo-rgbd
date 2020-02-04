function ypr = R2ypr(R)
% Extracts angles from a rotation matrix.
% yaw (Z) pitch (Y) roll (X) angles from rotation matrix.
% Extrinsic rotations; The XYZ system rotates, while xyz is fixed.
% Verified using MATLAB eul = rotm2eul(rotm, 'ZYX')

yaw = atan2(R(2,1),R(1,1));
pitch = atan2(-R(3,1),R(1,1)*cos(yaw)+R(2,1)*sin(yaw));
roll = atan2(R(3,2),R(3,3));

ypr = [yaw, pitch, roll];