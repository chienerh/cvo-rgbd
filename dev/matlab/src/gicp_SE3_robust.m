function T = gicp_SE3_robust(target, source, varargin)

% Create an ANN object of the target point cloud for NN queries
target_xyz = squeeze(double(target.Location));
source_xyz = squeeze(double(source.Location));
target_kdt = KDTreeSearcher(target_xyz);
source_kdt = KDTreeSearcher(source_xyz);

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
options.verbosity = 0;

% Initial guess
if nargin > 2
    T0 = varargin{1};
else
    T0 = [];
    T0.R = eye(3);
    T0.t = zeros(3,1);
end

% ICP loop: find correspondences and optimize
converged = false;
tf_epsilon = 1e-5; % 1e-6;
iter = 0;
maxIter = 50;
while ~converged && iter < maxIter
    % apply the current transformation to the source point cloud
    current_source = source_xyz * T0.R';
    current_source(:,1) = current_source(:,1) + T0.t(1);
    current_source(:,2) = current_source(:,2) + T0.t(2);
    current_source(:,3) = current_source(:,3) + T0.t(3);
    
    % ANN queries
    target_idx = knnsearch(target_kdt, current_source);
    
    p_source = source_xyz;
    p_target = target_kdt.X(target_idx,:);
    
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
    dR = zeros(3,3,store.n);
    dt = zeros(3,1,store.n);
    parfor i = 1:store.n
        dR(:,:,i) = -eloss(i) * Wrt{i} * p_source(i,:); 
        dt(:,:,i) = -eloss(i) * Wrt{i}; 
    end
    eg.R = sum(dR,3);
    eg.t = sum(dt,3);
end

function C = pc_covariances_ann(pckdt)
% Compute the empirical covariance at each point using an ANN search
    e = 1e-2; % covariance epsilon
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