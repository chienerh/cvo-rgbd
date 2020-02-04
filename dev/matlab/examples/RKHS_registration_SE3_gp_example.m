function out = RKHS_registration_SE3_gp_example()

    % load point target and source clouds
    pc1 = pcread('001000.pcd');
    pc2 = pcread('001001.pcd');

    % Regression  
    f1 = intensityfunc_reg_gp(pc1, 10);
    f2 = intensityfunc_reg_gp(pc2, 10);
    alpha = f1.post.alpha;
    beta = f2.post.alpha;    
    ell = exp(2*f1.gp.hyp.cov(1:3)); % lengscales
%     sf2 = exp(2*f1.rvm.hyp.cov(4)); 
    Linv = diag(1./ell);
    p_target = f1.gp.X;
    p_source = f2.gp.X;
    covfunc = f1.gp.covfunc;
    hyp = f1.gp.hyp.cov;

   % SE(3)
    M = specialeuclideanfactory(3);
    problem.M = M;
    % M.retr = M.retr2;
    M.retr = M.exp;
    problem.cost  = @cost;
    problem.egrad = @egrad;
%     problem.ehess = @ehess;
    options.maxiter = 20;
    
    % Initial guess
    T0 = [];
    T0.R = eye(3);
    T0.t = zeros(3,1);
    
    T1 = conjugategradient(problem, T0);
%     T1 = trustregions(problem, T0, options);

    % Numerically check gradient consistency
    figure;
    checkgradient(problem, T1);
    figure;
    checkhessian(problem, T1);
    
    out.f1 = f1;
    out.f2 = f2;
    out.T = T1;
    
    % Cost function
    function [f, store] = cost(X, store)
        
        store.z = (X.R' * p_source')' - (X.R' * X.t)';
        store.K = feval(covfunc{:}, hyp, p_target, store.z);
        f = -alpha' * store.K * beta;
    end

    % Define the Euclidean gradient of the cost function, that is, the
    % gradient of f(X) seen as a standard function of X.
    % We only need to give the Euclidean gradient. Manopt converts it
    % internally to the Riemannian counterpart.
    function [eg, store] = egrad(X, store)
        if ~isfield(store, 'K')
            [~, store] = cost(X, store);
        end
        eg.R = zeros(3);
        eg.t = zeros(3,1);
        [n, m] = size(store.K);
        for i = 1:n
            for j = 1:m
                d = p_target(i,:)' - store.z(j,:)';
                store.duR{i,j} = -p_source(j,1:3)' * d' * Linv + X.t * d' * Linv;
                store.dut{i,j} = (d' * Linv * X.R')';
                eg.R = eg.R + alpha(i) * beta(j) * store.K(i,j) * store.duR{i,j};
                eg.t = eg.t + alpha(i) * beta(j) * store.K(i,j) * store.dut{i,j};
            end
        end 
    end
end
