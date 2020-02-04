function H = Rx(roll)
% Rotation matrix about X axis (roll).

sr = sin(roll);
cr = cos(roll);

H = [1, 0, 0;
    0, cr, -sr;
    0, sr, cr];