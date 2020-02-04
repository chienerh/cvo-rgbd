clc; clear; close all

% Load input frame
ptCloud = pcread('001000.pcd');

% filter ranges greater than 20 meters
ptCloudOut = pcXYZIRangeFilter(ptCloud, 20);

% ptCloudOut = pcdownsample(ptCloudIn,'random',percentage)

pt_location = ptCloudOut.Location; % x y z coordinates
pt_intensity = ptCloudOut.Intensity / 255; % intensity [0, 1]

% downsample the pointcloud
skip = 250;
X = pt_location(1:skip:end,:);
y = pt_intensity(1:skip:end);

% Test points are all points
t = pt_location;

% measuring regression time
t0 = tic;

% GP setup
covfunc = {@covSEard}; hyp = []; hyp.cov = log([1.8096 1.6246 3.9090 4.8987]);
likfunc = @likGauss; sn = 0.1; hyp.lik = log(sn);

% optimize hyperparameters
hyp = minimize(hyp, @gp, -100, @infGaussLik, [], covfunc, likfunc, X, y);
% GP inference
[m, s2, ~, ~, ~, post] = gp(hyp, @infGaussLik, [], covfunc, likfunc, X, y, t);

% map the output into [0,1]
f = (m + abs(min(m))) / (max(m) - min(m));

time = toc(t0);

% plot the output
fsize = 20; % font size
set(groot, 'defaultAxesTickLabelInterpreter','latex'); 
set(groot, 'defaultLegendInterpreter','latex');

cmap = single(f .* repmat([255, 255, 255], length(f), 1)/ 255);
newptcloud = pointCloud(t, 'Intensity', f, 'Color', cmap);

figure; hold on
pcshow(newptcloud)
title('GP Intensity Regression')
axis equal, grid on
set(gca,'fontsize',fsize)
set(gca,'TickLabelInterpreter','latex')
figuresize(21,21,'cm')

orig_cmap = single(pt_intensity .* repmat([255, 255, 255], length(pt_intensity), 1)/ 255);
orig_ptcloud = pointCloud(t, 'Intensity', pt_intensity, 'Color', orig_cmap);

figure; hold on
pcshow(orig_ptcloud)
title('Original Point Cloud')
axis equal, grid on
set(gca,'fontsize',fsize)
set(gca,'TickLabelInterpreter','latex')
figuresize(21,21,'cm')