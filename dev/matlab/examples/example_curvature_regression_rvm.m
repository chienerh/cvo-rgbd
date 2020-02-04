clc; clear; close all

% Load input frame
ptCloud = pcread('001003.pcd');

% filter ranges greater than 20 meters
ptCloud = pcXYZIRangeFilter(ptCloud, 20);

gridSize = 0.1;
ptCloudOut = pcdownsample(ptCloud, 'gridAverage', gridSize);

pt_location = ptCloudOut.Location; % x y z coordinates

pt_curv = pc_curvature_ann(ptCloudOut);

% downsample the pointcloud
skip = 10;
X = pt_location(1:skip:end,:);
y = pt_curv(1:skip:end,:);

% Test points are all points
t = pt_location;

% measuring regression time
t0 = tic;

% RVM

iterations = 2000;

% Set up the options:
% - we set the diagnostics level to 2 
% - we will monitor the progress every 10 iterations
OPTIONS	= SB2_UserOptions('iterations',iterations,...
    'diagnosticLevel', 2,...
    'monitor', 10);

% Set initial parameter values:
% - this specification of the initial noise standard deviation is not
% necessary, but included here for illustration. If omitted, SPARSEBAYES
% will call SB2_PARAMETERSETTINGS itself to obtain an appropriate default
% for the noise (and other SETTINGS fields).
SETTINGS = SB2_ParameterSettings('NoiseStd',0.1);

% hyp trained by a GP
covfunc = {@covSEard}; hyp = []; hyp.cov = log([0.4470, 1.3649, 1.2287, 0.4005]);

BASIS = feval(covfunc{:}, hyp.cov, X);

% Run the main SPARSEBAYES function
[PARAMETER, HYPERPARAMETER, ~] = SparseBayes('Gaussian', BASIS, y, OPTIONS, SETTINGS);

w_infer	= PARAMETER.Value;

% Compute the inferred prediction function
test_BASIS = feval(covfunc{:}, hyp.cov, X(PARAMETER.Relevant, :), t)';
f = test_BASIS * w_infer;
% map the output into [0,1]
f_normalized = (f + abs(min(f))) / (max(f) - min(f));

time = toc(t0);

% plot the output
fsize = 20; % font size
set(groot, 'defaultAxesTickLabelInterpreter','latex'); 
set(groot, 'defaultLegendInterpreter','latex');

cmap = single(f_normalized .* repmat([255, 255, 255], length(f_normalized), 1)/ 255);
newptcloud = pointCloud(pt_location, 'Color', cmap);

figure; hold on
pcshow(newptcloud)
title('RVM Curvature Regression')
axis equal, grid on, colormap jet
set(gca,'fontsize',fsize)
set(gca,'TickLabelInterpreter','latex')
figuresize(21,21,'cm')

orig_cmap = single(pt_curv .* repmat([255, 255, 255], length(pt_curv), 1)/ 255);
orig_ptcloud = pointCloud(pt_location, 'Color', orig_cmap);

figure; hold on
pcshow(orig_ptcloud)
title('Original Point Cloud')
axis equal, grid on, colormap jet
set(gca,'fontsize',fsize)
set(gca,'TickLabelInterpreter','latex')
figuresize(21,21,'cm')