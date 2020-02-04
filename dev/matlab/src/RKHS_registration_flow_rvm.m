function T = RKHS_registration_flow_rvm(pc_target, pc_source)

    % Regression
    f1 = intensityfunc_reg_rvm(pc_target);
    f2 = intensityfunc_reg_rvm(pc_source);
    alpha = f1.parameters.Value;
    beta = f2.parameters.Value;
    ell = exp(2*f1.rvm.hyp.cov(1:3)); % lengscales
    sf2 = exp(2*f1.rvm.hyp.cov(4)); 
    Linv = diag(1./ell);
    p_target = f1.rvm.X;
    p_source = f2.rvm.X;
    covfunc = f1.rvm.covfunc;
    hyp = f1.rvm.hyp.cov;

    % SE(3)
    M = specialeuclideanfactory(3);
    problem.M = M;
    % M.retr = M.retr2;
    M.retr = M.exp;
    problem.cost  = @cost;
    problem.egrad = @egrad;
%     problem.ehess = @ehess;
%     options.maxiter = 50;
    options.verbosity = 1;
    
    % Initial guess
    T = eye(4);
    R = eye(3);
    t = zeros(3,1);
    
    converged = 0;
    e = 0.01;
    
    while ~converged
        store.z = (R * p_source')' + t';
        store.K = feval(covfunc{:}, hyp, p_target, store.z);
%         X = logm(T);
        A = zeros(length(alpha), length(beta));
        g = [];
        g.w = zeros(3,1);
        g.v = zeros(3,1);
        for i = 1:length(alpha)
            for j = 1:length(beta)
                A(i,j) = sf2 / norm(ell) * alpha(i) * beta(j) * store.K(i,j);
                g.w = g.w - A(i,j) * cross(p_target(i,1:3)', p_source(j,1:3)');
                g.v = g.v - A(i,j) * (p_target(i,1:3)' - p_source(j,1:3)');
            end
        end
        S = [0 -g.w(3) g.w(2)
            g.w(3) 0 -g.w(1)
            -g.w(2) g.w(1) 0];
        T = expm(e * [S, g.v; 0 0 0 0]) * T;
        R = T(1:3, 1:3);
        t = T(1:3,4);
        
%         xi = [X(3,2); X(1,3); X(2,1); X(1:3,4)];
%         dF = dot([g.w; g.v], xi);
        converged = norm([g.w; g.v]) < 1e-6;
%         f = -alpha' * store.K * beta;
    end

end
