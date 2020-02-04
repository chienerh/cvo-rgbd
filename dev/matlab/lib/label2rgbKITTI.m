function image = label2rgbKITTI(label)
% label to rgb conversion

KITTIcolormap = [[0.5020, 0.5020, 0.5020];
    [0.5020, 0, 0];
    [0.5020, 0.5020, 0];
    [0.2510, 0, 0.5020];
    [0.5020, 0.2510, 0.5020];
    [0.2510, 0.7529, 0];
    [0, 0, 0.7529];
    [0, 0.2510, 0.2510]];

[n, m] = size(label);
rgb = KITTIcolormap(label(:) + 1, :);  % 1st row in colormap corresponding to label 0
image = reshape(rgb, n, m, 3);

end

