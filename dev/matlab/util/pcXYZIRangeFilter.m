function ptCloudOut = pcXYZIRangeFilter(ptCloudIn, threshold)
% Remove all the points in ptCloudIn with range larger than threshold

% Calculate range for all points
if length(size(ptCloudIn.Location)) == 2
    pt_location = ptCloudIn.Location;
else
    pt_location = reshape(ptCloudIn.Location, [size(ptCloudIn.Location,2),3]);
end
if size(ptCloudIn.Intensity,1) == 1
    pt_intensity = ptCloudIn.Intensity';
else
    pt_intensity = ptCloudIn.Intensity;
end
pt_range = sqrt(sum(pt_location .* pt_location, 2));

% Filtering
pt_location(pt_range > threshold, :) = [];
pt_intensity(pt_range > threshold, :) = [];
ptCloudOut = pointCloud(pt_location, 'Intensity', pt_intensity);

end
