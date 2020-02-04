function curv = pc_curvature_ann(pc)
% Compute the curvature at each point using an ANN search
pckdt = KDTreeSearcher(pc.Location);
curv = zeros(pc.Count,1);
for i = 1:length(curv)
    nn_id = knnsearch(pckdt, pckdt.X(i,:), 'K', 6);
    Cov = cov(pckdt.X(nn_id,:));
    % GICP covariance
    [~, D] = eig(Cov);
    d = diag(D);
    sum_d = sum(d);
    if sum_d > 2*eps
        curv(i) = min(d) / sum_d;
    end
end
end