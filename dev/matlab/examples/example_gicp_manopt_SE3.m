function T = example_gicp_manopt_SE3()

% load point target and source clouds
pc1 = pcread('001000.pcd');
pc2 = pcread('001001.pcd');

% filter ranges greater than 20 meters
max_range = 10;
pc1 = pcXYZIRangeFilter(pc1, max_range);
pc2 = pcXYZIRangeFilter(pc2, max_range);

% downsample point clouds using grids
gridStep = 0.2;
target = pcdownsample(pc1,'gridAverage',gridStep);
source = pcdownsample(pc2,'gridAverage',gridStep);

% Create an ANN object of the target point cloud for NN queries
target_kdt = KDTreeSearcher(target.Location);
source_kdt = KDTreeSearcher(source.Location);

% Covariance normal at each point 
Ct = pc_covariances_ann(target_kdt);
Cs = pc_covariances_ann(source_kdt);

% Optmization setup
% SE(3)
M = specialeuclideanfactory(3);
problem.M = M;
% M.retr = M.retr2;
M.retr = M.exp;
problem.cost  = @cost;
problem.egrad = @egrad;
options.maxiter = 100;
options.verbosity = 1;

% Initial guess
T0 = [];
T0.R = eye(3);
T0.t = zeros(3,1);

% ICP loop: find correspondences and optimize
d_threshold = 1.5;
converged = false;
rot_epsilon = 1e-6;

while ~converged
    % apply the current transformation to the source point cloud
    current_source = (T0.R * source.Location')' + T0.t';
    
    % ANN queries
    idx = knnsearch(target_kdt, current_source);
    
    % apply distance threshold to remove outliers
    dist = sqrt(sum((current_source - target_kdt.X(idx,:)).^2,2));
    survived_idx = find(dist < d_threshold);
    target_idx = idx(dist < d_threshold);
    
    p_source = source.Location(survived_idx,:);
    p_target = target_kdt.X(target_idx,:);
    
    % solve for the new transformation
%     R1 = trustregions(problem, R0, options);
    T1 = conjugategradient(problem, T0, options);

    % check if converged
    if norm(logm([T0.R, T0.t; 0 0 0 1] \ [T1.R, T1.t; 0 0 0 1])) < rot_epsilon
        disp('Converged')
        converged = true;
    else
        T0 = T1;
        continue;
    end    
end

figure;
checkgradient(problem, T1);
% figure;
% checkhessian(problem, T1);

T = [T1.R, T1.t; 0 0 0 1];

% Cost function
function [f, store] = cost(X, store)
    % residual
    r = p_target - (X.R * p_source')' - X.t';
    store.r = r;
    % cost; sum of residuals
    f = 0;
    store.n = size(r,1);
    store.Winv = cell(store.n,1);
    for i = 1:store.n
        W = Ct{target_idx(i)} + X.R * Cs{survived_idx(i)} * X.R';
        store.Winv{i} = W \ eye(3);
        store.Wrt{i} = store.Winv{i} * r(i,:)';
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

function C = pc_covariances_ann(pckdt)
% Compute the empirical covariance at each point using an ANN search
    e = 0.01; % covariance epsilon
    C = cell(size(pckdt.X,1),1);
    for i = 1:length(C)
        nn_id = knnsearch(pckdt, pckdt.X(i,:), 'K', 6);
        % HACK: I'm adding a jitter to avoid singularity.
        Cov = cov(pckdt.X(nn_id,:)) + e * eye(3);
        % GICP covariance
        [V, D] = eig(Cov);
        D(1,1) = e;
        D(2,2) = 1;
        D(3,3) = 1;
        C{i} = V * D * V';
    end
end

end