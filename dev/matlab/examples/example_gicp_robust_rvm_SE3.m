function T = example_gicp_robust_rvm_SE3()

% ground truth

T0 = [-0.0388    0.0015    0.9992   66.9079
        -0.0059    1.0000   -0.0017   -8.6432   
        -0.9992   -0.0060   -0.0387  232.9343
        0   0   0   1];
    T1 = [-0.0400    0.0010    0.9992   67.5853   
        -0.0038    1.0000   -0.0011   -8.6618   
        -0.9992   -0.0039   -0.0400  232.9080
        0   0   0   1];
    
    T_gt = T0 \ T1;
    R_gt = T_gt(1:3,1:3);
    p_gt = T_gt(1:3,4);

% load point target and source clouds
pc1 = pcread('001000.pcd');
pc2 = pcread('001001.pcd');

% filter ranges greater than 20 meters
max_range = 10;
pc1 = pcXYZIRangeFilter(pc1, max_range);
pc2 = pcXYZIRangeFilter(pc2, max_range);

% Regression, target function
ft = intensityfunc_reg_rvm(pc1);
alpha = ft.parameters.Value;
ell = exp(2*ft.rvm.hyp.cov(1:3)); % lengscales
%     sf2 = exp(2*f1.rvm.hyp.cov(4));
Linv = diag(1./ell);
covfunc = ft.rvm.covfunc;
hyp = ft.rvm.hyp.cov;

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

disp('Ground truth Transformation:')
disp(T_gt)

% Cost function
function [f, store] = cost(X, store)
    R = X.R;
    t = X.t;
    % residual
    store.z = (R * p_source')' + t';
    store.K1 = feval(covfunc{:}, hyp, ft.rvm.X, p_target);
    store.K2 = feval(covfunc{:}, hyp, ft.rvm.X, store.z);
    r = p_target - store.z;
    store.r = r;
    % cost; sum of residuals
    f = 0;
    f1 = (store.K1' * alpha); % target intensity
    store.f2 = (store.K2' * alpha); % source intensity
    reg = (f1 - store.f2);     % functional regularizer
    lambda = 100;           % regularizer coefficient 
    store.n = size(r,1);
    store.Winv = cell(store.n,1);
    chauchy_alpha = 2; % Cauchy loss parameter
    eloss = zeros(size(r,1),1);
    parfor i = 1:store.n
        W = Ct{target_idx(i)} + R * Cs{i} * R';
        Wrt{i} = (W \ eye(3)) * r(i,:)';
        % i-th residual
        fi = .5 * r(i,:) * Wrt{i} + lambda * reg(i).^2;
        % Cauchy loss
        eloss(i) = 1/(fi / (chauchy_alpha^2) + 1);
        % cost
        f = f + (chauchy_alpha^2 * log(1 + fi/(chauchy_alpha^2)));
    end
    store.reg = reg;
    store.lambda = lambda;
    store.Wrt = Wrt;
    store.eloss = eloss;
end

% Euclidean gradient of cost function
function [eg, store] = egrad(X, store)
    if ~isfield(store, 'n')
        [~, store] = cost(X, store);
    end
    lambda = store.lambda;
    eloss = store.eloss;
    Wrt = store.Wrt;
    rvmX = ft.rvm.X;
    z  = store.z;
    K2 = store.K2;
    % Euclidean gradient
    egR = zeros(3,3,store.n);
    egt = zeros(3,1,store.n);
    % scalar coefficients from the first two step of the change rule
    dreg = -2 * (store.reg);
    parfor i = 1:store.n
        dR  = zeros(3,3);
        dt  = zeros(3,1);
        dXz = (Linv * (rvmX - z(i,:))')';
        alphaK2 = alpha .* K2(:,i);
        for j = 1:length(alpha)
           dR = dR + alphaK2(j) * dXz(j,:)' * p_source(i,:);
           dt = dt + alphaK2(j) * dXz(j,:)';
%            dR = dR + alpha(j) * store.K2(j,i) * Linv ...
%                * (ft.rvm.X(j,:)' - X.R * p_source(i,1:3)' - X.t) * p_source(i,1:3);
%            dt = dt + alpha(j) * store.K2(j,i) * Linv ...
%                * (ft.rvm.X(j,:)' - X.R * p_source(i,1:3)' - X.t);
        end
        egR(:,:,i) = -eloss(i) * (Wrt{i} * p_source(i,:) - lambda * dreg(i) * dR); 
        egt(:,:,i) = -eloss(i) * (Wrt{i} - lambda * dreg(i) * dt);
    end
    eg.R = sum(egR,3);
    eg.t = sum(egt,3);
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