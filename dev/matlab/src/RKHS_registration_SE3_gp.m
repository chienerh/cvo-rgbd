function T = RKHS_registration_SE3_gp(pc_target, pc_source)

    % Regression
    f1 = intensityfunc_reg_gp(pc_target);
    f2 = intensityfunc_reg_gp(pc_source);
    alpha = f1.post.alpha;
    beta = f2.post.alpha;   
    ell = exp(2*f1.gp.hyp.cov(1:3)); % lengscales
%     sf2 = exp(2*f1.gp.hyp.cov(4)); 
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
%     problem.egrad = @egrad;
%     problem.ehess = @ehess;
%     options.maxiter = 50;
    options.verbosity = 3;
    
    % Initial guess
    T0 = [];
    T0.R = eye(3);
    T0.t = zeros(3,1);
    
    T1 = conjugategradient(problem, T0, options);
%     T1 = trustregions(problem, T0, options);

    T = [T1.R T1.t; 0 0 0 1];
    
    figure;
    checkgradient(problem, T1);
    figure;
    checkhessian(problem, T1);
    
    % Cost function
    function [f, store] = cost(X, store)
        
%         store.z = (X.R * p_source')' + X.t';
%         store.K = feval(covfunc{:}, hyp, p_target, store.z);
%         f = -alpha' * store.K * beta;
        
        store.l = mean(1./ell);
        store.z = (X.R' * p_source')' - (X.R' * X.t)';
%         C = store.l * sq_dist(p_target', store.z');
        store.K = feval(covfunc{:}, hyp, p_target, store.z);
        f = -alpha' * store.K * beta;


        
%         T = [X.R' -X.R'*X.t; 0 0 0 1];
%         f = 0;
%         n = length(alpha);
%         m = length(beta);
%         for i = 1:n
%             for j = 1:m
%                 % weight
%                 w = alpha(i) * beta(j) * store.l;
%                 % residual
%                 r = p_target(i,:)' - (X.R' * p_source(j,1:3)' - X.R'*X.t);
%                 % cost; weighted norm of residual
%                 f = f + w * (r' * r);
%             end
%         end
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
        n = length(alpha);
        m = length(beta);
        for i = 1:n
            for j = 1:m
                dR = Linv * cross(p_target(i,:)', store.z(j,:)');
                dt = Linv * store.z(j,:)' - p_target(i,:)';
                eg.R = eg.R - 2 * alpha(i) * beta(j) * dR;
                eg.t = eg.t - 2 * alpha(i) * beta(j) * dt;
            end
        end 
    end
end

function [Ax] = skew(v)
% Convert from vector to skew symmetric matrix
Ax = [    0, -v(3),  v(2);
       v(3),     0, -v(1);
      -v(2),  v(1),     0];
end
