function out = RKHS_registration_SO3_gp_example()

    % Regression  
    f1 = intensityfunc_reg_gp_centralize('001000.pcd', 20);
    f2 = intensityfunc_reg_gp_centralize('001001.pcd', 20);
    alpha = f1.post.alpha;
    beta = f2.post.alpha;    
    ell = exp(2*f1.gp.hyp.cov(1:3)); % lengscales
%     sf2 = exp(2*f1.rvm.hyp.cov(4)); 
    Linv = diag(1./ell);
    p_target = f1.gp.X;
    p_source = f2.gp.X;
    covfunc = f1.gp.covfunc;
    hyp = f1.gp.hyp.cov;

    % SO(3)
    M = rotationsfactory(3);
    problem.M = M;
    % M.retr = M.retr2;
    M.retr = M.exp;
    problem.cost  = @cost;
    problem.egrad = @egrad;
    options.maxiter = 20;
    
    % Initial guess
    R0 = eye(3);
    
    R1 = conjugategradient(problem, R0);
%     R1 = trustregions(problem, R0, options);

    % Numerically check gradient consistency
    figure;
    checkgradient(problem, R1);
    figure;
    checkhessian(problem, R1);
    
    out.f1 = f1;
    out.f2 = f2;
    out.T.R = R1;
    out.T.t = f1.pc_mean' - R1 * f2.pc_mean';
    
    % Cost function
    function [f, store] = cost(X, store)
        
        store.z = (X * p_source(:,1:3)')';
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
        eg = zeros(3,3);
        [n, m] = size(store.K);
        for i = 1:n
            for j = 1:m
                d = p_target(i,:)' - store.z(j,:)';
                store.du{i,j} = Linv * d * p_source(j,1:3);
                eg = eg - alpha(i) * beta(j) * store.K(i,j) * store.du{i,j};
            end
        end 
    end
end
