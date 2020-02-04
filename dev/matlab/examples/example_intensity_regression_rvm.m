clc; clear; close all

% Load input frame
ptCloud = pcread('001001.pcd');

% filter ranges greater than 20 meters
ptCloudOut = pcXYZIRangeFilter(ptCloud, 20);

% ptCloudOut = pcdownsample(ptCloudIn,'random',percentage)

pt_location = ptCloudOut.Location; % x y z coordinates
pt_intensity = ptCloudOut.Intensity / 255; % intensity [0, 1]

% downsample the pointcloud
skip = 200;
X = pt_location(1:skip:end,:);
y = double(pt_intensity(1:skip:end));

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
    'diagnosticLevel', 1,...
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

% use sparse multiplication
% BASIS(BASIS < 1e-5) = 0;
% BASIS = sparse(double((BASIS + BASIS')/2));

% Run the main SPARSEBAYES function
[PARAMETER, HYPERPARAMETER, ~] = SparseBayes('Gaussian', BASIS, y, OPTIONS, SETTINGS);

w_infer	= PARAMETER.Value;

% Compute the inferred prediction function
test_BASIS = feval(covfunc{:}, hyp.cov, X(PARAMETER.Relevant, :), t)';
y_out = test_BASIS * w_infer;
% map the output into [0,1]
f = (y_out + abs(min(y_out))) / (max(y_out) - min(y_out));

time = toc(t0);

% plot the output
fsize = 20; % font size
set(groot, 'defaultAxesTickLabelInterpreter','latex'); 
set(groot, 'defaultLegendInterpreter','latex');

cmap = single(f .* repmat([255, 255, 255], length(f), 1)/ 255);
newptcloud = pointCloud(t, 'Intensity', f, 'Color', cmap);

figure; hold on
pcshow(newptcloud)
plot3(X(PARAMETER.Relevant, 1), X(PARAMETER.Relevant, 2), X(PARAMETER.Relevant, 3),'rs')
title('RVM Intensity Regression')
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