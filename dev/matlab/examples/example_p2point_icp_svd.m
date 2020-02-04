function T1 = point2point_icp_svd()

% load point target and source clouds
pc1 = pcread('001000.pcd');
pc2 = pcread('001001.pcd');

% filter ranges greater than 20 meters
max_range = 20;
pc1 = pcXYZIRangeFilter(pc1, max_range);
pc2 = pcXYZIRangeFilter(pc2, max_range);

% downsample point clouds using grids
gridStep = 0.05;
target = pcdownsample(pc1,'gridAverage',gridStep);
source = pcdownsample(pc2,'gridAverage',gridStep);

% create an ANN object of the target point cloud for NN queries
target_kdt = KDTreeSearcher(target.Location);

% ICP loop: find correspondences and optimize
T0 = eye(4); % initial tf
d_threshold = 1; % meters
converged = false;
tf_epsilon = 1e-4;

while ~converged
    % apply the current transformation to the source point cloud
    current_source = (T0 * [source.Location, ones(source.Count,1)]')';
    current_source = current_source(:,1:3);
    
    % ANN queries
    idx = knnsearch(target_kdt, current_source);
    
    % apply distance threshold to remove outliers
    dist = sqrt(sum((current_source - target_kdt.X(idx,:)).^2,2));
    survived_idx = dist < d_threshold;
    p_source = source.Location(survived_idx,:);
    p_target = target_kdt.X(idx(survived_idx),:);
    
    % solve for the new transformation
    [R, t] = get_transformation(p_target', p_source');
    
    T1 = [R, t; 0 0 0 1];
    disp(T1)
    disp(norm(logm(T0 \ T1)))
    % check if converged
    if norm(logm(T0 \ T1)) < tf_epsilon
        disp('Converged')
        converged = true;
    else
        T0 = T1;
        continue;
    end    
end

function [Rot, tran] = get_transformation(P, X)
%GET_TRANSFORMATION compute registration vector of data P to model X
%   P and X are 3xN matrixes for N points in 3 dimensions, i.e. each column
%   corresponds to a single point. See Besl & McKay (1992) section III.C.

mu_p = sum(P,2)/size(P,2);
mu_x = sum(X,2)/size(X,2);

Sigma = (P*X')/size(P,2) - mu_p*mu_x';

A = (Sigma - Sigma');
Delta = [A(2,3) A(3,1) A(1,2)]';

tr = trace(Sigma);
Q = [tr Delta'; Delta (Sigma + Sigma' - tr*eye(3))];

[V,D] = eig(Q);
[~,index] = max(diag(D));
q_r = V(:,index);

Rot = quat2dcm(q_r')';
tran = mu_x - Rot * mu_p;
end

end