function T = gicp_rvm_SE3(target, source, varargin)

% Regression, target function
ft = intensityfunc_reg_rvm(target);
alpha = ft.parameters.Value;
ell = exp(2*ft.rvm.hyp.cov(1:3)); % lengscales
%     sf2 = exp(2*f1.rvm.hyp.cov(4));
Linv = diag(1./ell);
covfunc = ft.rvm.covfunc;
hyp = ft.rvm.hyp.cov;

% downsample point clouds using grids
gridStep = 0.2;
target = pcdownsample(target,'gridAverage',gridStep);
source = pcdownsample(source,'gridAverage',gridStep);

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
d_threshold = 1.0; % 1.5;
converged = false;
tf_epsilon = 1e-5; % 1e-6;
iter = 0;
maxIter = 50;
while ~converged && iter < maxIter
    % apply the current transformation to the source point cloud
    current_source = (T0.R * source_xyz')' + T0.t';
    
    % ANN queries
    idx = knnsearch(target_kdt, current_source);
    
    % apply distance threshold to remove outliers
    dist = sqrt(sum((current_source - target_kdt.X(idx,:)).^2,2));
    survived_idx = find(dist < d_threshold);
    target_idx = idx(dist < d_threshold);
    
    p_source = source_xyz(survived_idx,:);
    p_target = target_kdt.X(target_idx,:);
    
    % solve for the new transformation
%     R1 = trustregions(problem, R0, options);
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
    store.z = (X.R * p_source')' + X.t';
    store.K1 = feval(covfunc{:}, hyp, ft.rvm.X, p_target);
    store.K2 = feval(covfunc{:}, hyp, ft.rvm.X, store.z);
    r = p_target - store.z;
    store.r = r;
    % cost; sum of residuals
    f = 0;
    f1 = (store.K1' * alpha); % target intensity
    store.f2 = (store.K2' * alpha); % source intensity
    store.reg = (f1 - store.f2);     % functional regularizer
    store.lambda = 100;           % regularizer coefficient 
    store.n = size(r,1);
    store.Winv = cell(store.n,1);
    for i = 1:store.n
        W = Ct{target_idx(i)} + X.R * Cs{survived_idx(i)} * X.R';
        store.Winv{i} = W \ eye(3);
        store.Wrt{i} = store.Winv{i} * r(i,:)';
        f = f + .5 * r(i,:) * store.Wrt{i} + store.lambda * store.reg(i).^2;
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
    % scalar coefficients from the first two step of the change rule
    dreg = -2 * (store.reg);
    for i = 1:store.n
        dR  = zeros(3,3);
        dt  = zeros(3,1);
        dXz = (Linv * (ft.rvm.X - store.z(i,:))')';
        alphaK2 = alpha .* store.K2(:,i);
        for j = 1:length(alpha)
           dR = dR + alphaK2(j) * dXz(j,:)' * p_source(i,1:3);
           dt = dt + alphaK2(j) * dXz(j,:)';
%            dR = dR + alpha(j) * store.K2(j,i) * Linv ...
%                * (ft.rvm.X(j,:)' - X.R * p_source(i,1:3)' - X.t) * p_source(i,1:3);
%            dt = dt + alpha(j) * store.K2(j,i) * Linv ...
%                * (ft.rvm.X(j,:)' - X.R * p_source(i,1:3)' - X.t);
        end
        eg.R = eg.R - store.Wrt{i} * p_source(i,:) + store.lambda * dreg(i) * dR; 
        eg.t = eg.t - store.Wrt{i} + store.lambda * dreg(i) * dt;
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