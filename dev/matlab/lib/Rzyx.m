function R = Rzyx(yaw,pitch,roll)
% Rotation matrix for yaw pitch roll angles
% Order of rotation R = Rz * Ry * Rx
% Extrinsic rotations; The XYZ system rotates, while xyz is fixed.
% Verified using MATLAB rotm = eul2rotm([yaw,pitch,roll], 'ZYX')

R = Rz(yaw) * Ry(pitch) * Rx(roll);