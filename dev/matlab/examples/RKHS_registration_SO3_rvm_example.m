function out = RKHS_registration_SO3_rvm_example()

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
    
    % Regression
    f1 = intensityfunc_reg_rvm_centralize('001000.pcd', 40);
    f2 = intensityfunc_reg_rvm_centralize('001001.pcd', 40);
    alpha = f1.parameters.Value;
    beta = f2.parameters.Value;
    ell = exp(2*f1.rvm.hyp.cov(1:3)); % lengscales
%     sf2 = exp(2*f1.rvm.hyp.cov(4)); 
    Linv = diag(1./ell);
    p_target = f1.rvm.X;
    p_source = f2.rvm.X;
    covfunc = f1.rvm.covfunc;
    hyp = f1.rvm.hyp.cov;

    % SO(3)
    M = rotationsfactory(3);
    problem.M = M;
    % M.retr = M.retr2;
    M.retr = M.exp;
    problem.cost  = @cost;
    problem.egrad = @egrad;
    problem.ehess = @ehess;
%     problem.ehess = @ehess;
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

%     pc1 = pcread('001000.pcd');
%     pc2 = pcread('001001.pcd');
%     tform = pcregrigid(pc2, pc1, 'Metric','pointToPlane');
%     T_matlab = tform.T';
    
    out.f1 = f1;
    out.f2 = f2;
    out.T.R = R1;
    out.T.t = f1.pc_mean' - R1 * f2.pc_mean';
%     out.xcost = xcost;
%     out.info = info;
%     out.options = options;
    out.T_gt.R = R_gt;
    out.T_gt.t = p_gt;
%     out.T_matlab.R = T_matlab(1:3,1:3);
%     out.T_matlab.t = T_matlab(1:3,4);
    
    % Cost function
    function [f, store] = cost(X, store)
        
        store.z = (X * p_source')';
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

    % Euclidean Hessian of cost function
    function [eh, store] = ehess(X, Xdot, store)
        if ~isfield(store, 'K')
            [~, store] = cost(X, store);
        end
        % Euclidean Hessian
        eh = zeros(3,3);
        [n, m] = size(store.K);
        for i = 1:n
            for j = 1:m
                eh = eh - alpha(i) * beta(j) * store.K(i,j) * ...
                    (store.du{i,j} * store.du{i,j} - Linv * p_source(j,1:3)' * p_source(j,1:3));
            end
        end
    end
end
