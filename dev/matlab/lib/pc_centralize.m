function [pmean, p2] = pc_centralize(p)

n = cast(size(p, 1), 'like', p);

% Find data centroid and deviations from centroid
pmean = sum(p,1)/n;
p2 = bsxfun(@minus, p, pmean);

end