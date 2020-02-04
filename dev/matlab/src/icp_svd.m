function T = icp_svd(target, source)

% Create an ANN object of the target point cloud for NN queries
target_xyz = double(target.Location);
source_xyz = double(source.Location);
target_kdt = KDTreeSearcher(target_xyz);


% Initial guess
T0 = eye(4);

% ICP loop: find correspondences and optimize
d_threshold = 1.5;
converged = false;
tf_epsilon = 1e-6;
iter = 0;
maxIter = 100;
while ~converged && iter < maxIter
    % apply the current transformation to the source point cloud
    current_source = source_xyz * T0(1:3, 1:3)';
    current_source(:,1) = current_source(:,1) + T0(1,4);
    current_source(:,2) = current_source(:,2) + T0(2,4);
    current_source(:,3) = current_source(:,3) + T0(3,4);
    
    % ANN queries
    idx = knnsearch(target_kdt, current_source);
    
    % apply distance threshold to remove outliers
    dist = sqrt(sum((current_source - target_kdt.X(idx,:)).^2,2));
    survived_idx = dist < d_threshold;
    p_source = source_xyz(survived_idx,:);
    p_target = target_kdt.X(idx(survived_idx),:);
    
    % solve for the new transformation
    T1 = get_transformation(p_source', p_target');
    
    % check if converged
    if norm(logm(T0 \ T1)) < tf_epsilon
        disp('Converged')
        converged = true;
    else
        T0 = T1;
        iter = iter + 1;
        if ~(iter < maxIter)
            disp(['Not converged. Maximum iteration of ', num2str(maxIter), ' is reached'])
        end
        continue;
    end    
end

T = T1;

function A = get_transformation(P, X)
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

R = quat2dcm(q_r')';
t = mu_x - R * mu_p;

A = [R, t; 0 0 0 1];
end

end