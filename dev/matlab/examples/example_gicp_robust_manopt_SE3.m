function T = example_gicp_robust_manopt_SE3()

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
converged = false;
rot_epsilon = 1e-6;

while ~converged
    % apply the current transformation to the source point cloud
    current_source = (T0.R * source.Location')' + T0.t';
    
    % ANN queries
    target_idx = knnsearch(target_kdt, current_source);
    
    p_source = source.Location;
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
    R = X.R;
    t = X.t;
    % residual
    r = p_target - p_source * R';
    r(:,1) = r(:,1) - t(1);
    r(:,2) = r(:,2) - t(2);
    r(:,3) = r(:,3) - t(3);
    
    store.r = r;
    % cost; sum of residuals
    f = 0;
    chauchy_alpha = 2; % Cauchy loss parameter
    store.n = size(r,1);
    eloss = zeros(size(r,1),1);
    Wrt = cell(store.n,1);
    parfor i = 1:store.n
        W = Ct{target_idx(i)} + R * Cs{i} * R';
        Wrt{i} = (W \ eye(3)) * r(i,:)';
        % i-th residual
        fi = .5 * r(i,:) * Wrt{i};
        % Cauchy loss
        eloss(i) = 1/(fi / (chauchy_alpha^2) + 1);
        % cost
        f = f + (chauchy_alpha^2 * log(1 + fi/(chauchy_alpha^2)));
    end
    store.Wrt = Wrt;
    store.eloss = eloss;
end

% Euclidean gradient of cost function
function [eg, store] = egrad(X, store)
    if ~isfield(store, 'n')
        [~, store] = cost(X, store);
    end
    eloss = store.eloss;
    Wrt = store.Wrt;
    % Euclidean gradient
    dR = zeros(3,3);
    dt = zeros(3,1);
    parfor i = 1:store.n
        dR = dR - eloss(i) * Wrt{i} * p_source(i,:); 
        dt = dt - eloss(i) * Wrt{i}; 
    end
    eg.R = dR;
    eg.t = dt;
end

function C = pc_covariances_ann(pckdt)
% Compute the empirical covariance at each point using an ANN search
    e = 0.01; % covariance epsilon
    C = cell(size(pckdt.X,1),1);
    parfor i = 1:length(C)
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