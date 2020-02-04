function I = feature_inner_product(Lx, Lz, scale)
% Inner product matrix in feature space.
% Lx and Lz are n-by-d and m-by-d feature matrices of each point cloud,
% respectively, where n and m are number of observations.
% The resulting I matrix, is n-by-m and contains the inner product of all
% combinations of points from X and Z.
% scale is the parameter to scale the value of the inner product.

c = 0.5; % polynomial constant term
d = 2;
% quadratic polynomial feature map
% phi = @(X) [X.^2, sqrt(2) * X(:,3) .* X(:,2), ...
%     sqrt(2) * X(:,3) .* X(:,1), sqrt(2) * X(:,2) .* X(:,1), ...
%     sqrt(2*c) * X, c*ones(size(X,1), 1)];
% 
% Phi_x = phi(Lx);
% Phi_z = phi(Lz);

%  Each column of I is n-by-1 vector of inner product of all Phi_x rows with a
%  row of Phi_z. For example for the i-th column of I, we need
% <Phi_x, Phi_zi'> = Phi_x * Phi_zi'. We can compute I all at once using Phi_x * Phi_z'.
% I = scale * double(Phi_x) * double(Phi_z)';
I = scale * (double(Lx) * double(Lz)' + c).^d;
end