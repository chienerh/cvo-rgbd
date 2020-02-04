function T1 = rkhs_toy_example_3D()
clc; clear all; close all

% kernel = @(x,z) exp(-norm(x-z)^2);
% 
% alpha = [-3 -4 2.5 -2 1 -5 3 1 3 -2 4 2 3];
% beta = [-3 -4 1.5 -2 1 -5 3 1 3 -2 4 2 3];
% 
% d = 1;
% xt = linspace(0,3, 15)';
% [X1t, X2t] = meshgrid(xt, xt);
% X3t = exp(-X1t.^2 - X2t.^2);
% Xt = [X1t(:), X2t(:), X3t(:)];
% 
% xs = linspace(0 + d,3 + d, 10)';
% [X1s, X2s] = meshgrid(xs, xs);
% X3s = exp(-X1s.^2 - X2s.^2);
% Xs = [X1s(:), X2s(:), X3s(:)];
% 
% % pick random basis!
% basis_idt = floor(size(Xt,1) * rand(length(alpha),1)) + 1;
% 
% yt = [];
% for j = 1:size(Xt,1)
%     for i = 1:length(alpha)
%         yt(j,1) = alpha(i) * kernel(Xt(j,:)', Xt(basis_idt(i),:)');
%     end
% end
% 
% % pick random basis!
% basis_ids = floor(size(Xs,1) * rand(length(beta),1)) + 1;
% 
% ys = [];
% for j = 1:size(Xs,1)
%     for i = 1:length(beta)
%         ys(j,1) = beta(i) * kernel(Xs(j,:)', Xs(basis_ids(i),:)');
%     end
% end
% 
% figure; hold on
% plot3(Xt(:,1), Xt(:,2), Xt(:,3), 's', Xs(:,1), Xs(:,2), Xs(:,3), 'o', 'markersize', 8)
% surf(X1t, X2t, X3t, reshape(yt, size(X1t)), 'FaceAlpha',0.5)
% surf(X1s, X2s, X3s, reshape(ys, size(X1s)), 'FaceAlpha',0.5)
% axis auto, grid on, view(-115, 45)
% 
% p_target = Xt; % target (fixed)
% p_source = Xs; % source (moving)

% load point target and source clouds
pc1 = pcread('001008.pcd');
pc2 = pcread('001012.pcd');

% filter ranges greater than 20 meters
max_range = 8;
pc1 = pcXYZIRangeFilter(pc1, max_range);
pc2 = pcXYZIRangeFilter(pc2, max_range);

% Regression, target function
ft = intensityfunc_reg_rvm(pc1);
alpha = ft.parameters.Value;
ell = exp(2*ft.rvm.hyp.cov(1:3)); % lengscales
covfunc = ft.rvm.covfunc;
hyp = ft.rvm.hyp.cov;
p_target = ft.rvm.X;

% Regression, source function
fs = intensityfunc_reg_rvm(pc2);
beta = fs.parameters.Value;
p_source = fs.rvm.X;

figure; hold on
cmap = single(ft.f .* repmat([255, 255, 255], length(ft.f), 1)/ 255);
newptcloud = pointCloud(double(squeeze(pc1.Location)), 'Intensity', ft.f, 'Color', cmap);
pcshow(newptcloud)
cmap = single(fs.f .* repmat([255, 255, 255], length(fs.f), 1)/ 255);
newptcloud = pointCloud(double(squeeze(pc2.Location)), 'Intensity', fs.f, 'Color', cmap);
pcshow(newptcloud)

% Optmization setup
% SE(3)
M = specialeuclideanfactory(3);
problem.M = M;
% M.retr = M.retr2;
M.retr = M.exp;
problem.cost  = @cost;
options.maxiter = 100;
options.verbosity = 2;

% Initial guess
T0 = [];
T0.R = eye(3);
T0.t = zeros(3,1);

T1 = conjugategradient(problem, T0, options);
   
% Xs = (T1.R' * Xs')' - (T1.R' * T1.t)';
% X1s = reshape(Xs(:,1), size(X1s));
% X2s = reshape(Xs(:,2), size(X2s));
% X3s = reshape(Xs(:,3), size(X3s));
% 
% figure; hold on
% plot3(Xt(:,1), Xt(:,2), Xt(:,3), 's', Xs(:,1), Xs(:,2), Xs(:,3), 'o', 'markersize', 8)
% surf(X1t, X2t, X3t, reshape(yt, size(X1t)), 'FaceAlpha',0.5)
% surf(X1s, X2s, X3s, reshape(ys, size(X1s)), 'FaceAlpha',0.5)
% axis auto, grid on, view(-115, 45)

figure; hold on
cmap = single(ft.f .* repmat([255, 255, 255], length(ft.f), 1)/ 255);
newptcloud = pointCloud(double(squeeze(pc1.Location)), 'Intensity', ft.f, 'Color', cmap);
pcshow(newptcloud)

loc = double(squeeze(pc2.Location));
loc = (T1.R' * loc')' - (T1.R' * T1.t)';

cmap = single(fs.f .* repmat([255, 255, 255], length(fs.f), 1)/ 255);
newptcloud = pointCloud(loc, 'Intensity', fs.f, 'Color', cmap);
pcshow(newptcloud)

figure;
checkgradient(problem, T1);
% figure;
% checkhessian(problem, T1);

% Cost function
    function f = cost(X)
        z = (X.R' * p_source')' - (X.R' * X.t)';
        for ii = 1:length(alpha)
            for jj = 1:length(beta)
%                 f = -alpha(ii) * beta(jj) * kernel(p_target(basis_idt(ii),:), z(basis_ids(jj),:));
                f = -alpha(ii) * beta(jj) * feval(covfunc{:}, hyp, p_target(ii,:), z(jj,:));
            end
        end
    end

end
