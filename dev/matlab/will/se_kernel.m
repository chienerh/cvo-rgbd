function K = se_kernel(X, Z, l, s2)
% Isotropic (same length-scale for all dimensions) squared-exponential kernel
% X and Z are n-by-d and m-by-d inputs matrices, respectively, where n and
% m are number of observations and d is the dimension of the input space.
% The resulting kernel matrix, K, is n-by-m.
% l and s2 are hyperparameters, i.e., length-scale and signal
% variance, respectively.

% number of observation for each input
nx	= size(X,1);
nz	= size(Z,1);

% compute matrix of pair-wise squared distances
D2 = (sum((X.^2), 2) * ones(1,nz)) + (ones(nx, 1) * sum((Z.^2),2)') - ...
    2 * X * Z';

% Isotropic SE kernel
K = s2 * exp(-D2 / (2*l^2));