function T = icp_p2plane_SE3(target, source)

% surface normal at each point 
target_normals = pcnormals(target);

% Create an ANN object of the target point cloud for NN queries
target_xyz = double(target.Location);
source_xyz = double(source.Location);
target_kdt = KDTreeSearcher(target_xyz);

% Optmization setup
% SE(3)
M = specialeuclideanfactory(3);
problem.M = M;
% M.retr = M.retr2;
% M.retr = M.exp;
problem.cost  = @cost;
problem.egrad = @egrad;
options.maxiter = 5;
options.verbosity = 0;

% Initial guess
T0 = [];
T0.R = eye(3);
T0.t = zeros(3,1);

% ICP loop: find correspondences and optimize
d_threshold = 1.5;
converged = false;
tf_epsilon = 1e-6;
iter = 0;
maxIter = 20;
while ~converged && iter < maxIter
    % apply the current transformation to the source point cloud
    current_source = source_xyz * T0.R';
    current_source(:,1) = current_source(:,1) + T0.t(1);
    current_source(:,2) = current_source(:,2) + T0.t(2);
    current_source(:,3) = current_source(:,3) + T0.t(3);
    
    % ANN queries
    idx = knnsearch(target_kdt, current_source);
    
    % apply distance threshold to remove outliers
    dist = sqrt(sum((current_source - target_kdt.X(idx,:)).^2,2));
    survived_idx = dist < d_threshold;
    p_source = source_xyz(survived_idx,:);
    p_target = target_kdt.X(idx(survived_idx),:);
    normals = target_normals(idx(survived_idx),:);
    
    % solve for the new transformation
%     T1 = trustregions(problem, T0, options);
    T1 = conjugategradient(problem, T0, options);
    
    % check if converged
    if norm(logm([T0.R, T0.t; 0 0 0 1] \ [T1.R, T1.t; 0 0 0 1])) < tf_epsilon
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

T = [T1.R, T1.t; 0 0 0 1];

% Cost function
function [f, store] = cost(X, store)
    % residual
    r = p_target - p_source * X.R';
    r(:,1) = r(:,1) - X.t(1);
    r(:,2) = r(:,2) - X.t(2);
    r(:,3) = r(:,3) - X.t(3);
    
    % cost; sum of residuals
    f = 0;
    store.n = size(r,1);
    for i = 1:store.n
        store.W = normals(i,:)' * normals(i,:);
        store.Wrt{i} = store.W * r(i,:)';
        f = f + .5 * r(i,:) * store.Wrt{i};
    end
end

% Euclidean gradient of cost function
function [eg, store] = egrad(X, store)
    if ~isfield(store, 'n')
        [~, store] = cost(X, store);
    end
    % Euclidean gradient
    eg.R = zeros(3,3);
    eg.t = zeros(3,1);
    for i = 1:store.n
        eg.R = eg.R - store.Wrt{i} * p_source(i,:); 
        eg.t = eg.t - store.Wrt{i}; 
    end
end

end