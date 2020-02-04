function T = gicp_fmin(target, source)

% Create an ANN object of the target point cloud for NN queries
target_xyz = double(target.Location);
source_xyz = double(source.Location);
target_kdt = KDTreeSearcher(target_xyz);
source_kdt = KDTreeSearcher(source_xyz);

% Covariance normal at each point 
Ct = pc_covariances_ann(target_kdt);
Cs = pc_covariances_ann(source_kdt);

% Initial guess
T0 = [1 0 0 0 0 0 0];

% ICP loop: find correspondences and optimize
d_threshold = 1.5;
converged = false;
tf_epsilon = 1e-6;
iter = 0;
maxIter = 20;
while ~converged && iter < maxIter
    % apply the current transformation to the source point cloud
    T0(1:4) = quatnormalize(T0(1:4));
    Rot = quat2dcm(T0(1:4));
    tran = T0(5:7)';
    current_source = source_xyz * Rot';
    current_source(:,1) = current_source(:,1) + tran(1);
    current_source(:,2) = current_source(:,2) + tran(2);
    current_source(:,3) = current_source(:,3) + tran(3);
    
    % ANN queries
    idx = knnsearch(target_kdt, current_source);
    
    % apply distance threshold to remove outliers
    dist = sqrt(sum((current_source - target_kdt.X(idx,:)).^2,2));
    survived_idx = find(dist < d_threshold);
    target_idx = idx(dist < d_threshold);
    
    p_source = source_xyz(survived_idx,:);
    p_target = target_kdt.X(target_idx,:);
    
    % solve for the new transformation
    T1 = fminsearch(@cost, T0 , struct('Display', 'final', 'TolFun', 1e-4, 'TolX',1e-1));
%     T1 = fminunc(@cost, T0);
    
    % check if converged
    current_tf_epsilon = norm(logm([quat2dcm(quatnormalize(T0(1:4))), T0(5:7)'; 0 0 0 1] \ [quat2dcm(quatnormalize(T1(1:4))), T1(5:7)'; 0 0 0 1]));
    disp(current_tf_epsilon)
    
    if current_tf_epsilon < tf_epsilon
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

Rot = quat2dcm(quatnormalize(T1(1:4)));
tran = T1(5:7)';
T = [Rot, tran; 0 0 0 1];

% Cost function
function f = cost(X)
    R = quat2dcm(quatnormalize(X(1:4)));
    t = X(5:7)';
    % residual
    r = p_target - p_source * R';
    r(:,1) = r(:,1) - t(1);
    r(:,2) = r(:,2) - t(2);
    r(:,3) = r(:,3) - t(3);
    
    % cost; sum of residuals
    f = 0;
    for i = 1:size(r,1)
        W = Ct{target_idx(i)} + R * Cs{survived_idx(i)} * R';
        f = f + .5 * r(i,:) * (W \ r(i,:)');
    end
end

function C = pc_covariances_ann(pckdt)
% Compute the empirical covariance at each point using an ANN search
    e = 1e-2; % covariance epsilon
    C = cell(size(pckdt.X,1),1);
    for i = 1:length(C)
        nn_id = knnsearch(pckdt, pckdt.X(i,:), 'K', 6);
        Cov = cov(pckdt.X(nn_id,:));
        % GICP covariance
        [V, D] = eig(Cov);
        D(1,1) = e;
        D(2,2) = 1;
        D(3,3) = 1;
        C{i} = V * D * V';
    end
end

end