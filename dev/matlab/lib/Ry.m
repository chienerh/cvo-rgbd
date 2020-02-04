function H = Ry(pitch)
% Rotation matrix about Y axis (pitch).

sp = sin(pitch);
cp = cos(pitch);

H = [cp, 0, sp;
    0, 1, 0;
    -sp, 0, cp];