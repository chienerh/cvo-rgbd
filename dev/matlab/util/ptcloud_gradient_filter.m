function cloud_out = ptcloud_gradient_filter(cloud_in, idx)

X = reshape(cloud_in.Location,[], 3, 1);
C = reshape(cloud_in.Color,[], 3, 1);
X = X(reshape(idx, [], 1), :);
C = C(reshape(idx, [], 1), :);
C = C(~isnan(X(:,1)),:);
X = X(~isnan(X(:,1)),:);

cloud_out = pointCloud(X, 'Color', C);