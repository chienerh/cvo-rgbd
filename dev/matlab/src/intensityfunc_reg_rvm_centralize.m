function out = intensityfunc_reg_rvm_centralize(pcname, varargin)

% Load input frame
ptCloud = pcread(pcname);

if nargin > 1 && isnumeric(varargin{1}) && ~any(isnan(varargin{1})) && ~isempty(varargin{1})
    max_range =  varargin{1};
else
    warning('The second input is the range limit; the default value of 20 meters is set.')
    max_range =  20;
end

% filter ranges greater than 20 meters
ptCloudOut = pcXYZIRangeFilter(ptCloud, max_range);

% Center point clouds at origin
[pc_mean, pc_new] = pc_centralize(ptCloudOut.Location);

% ptCloudOut = pcdownsample(ptCloudIn,'random',percentage)

pt_location = pc_new; % x y z coordinates
pt_intensity = ptCloudOut.Intensity / 255; % intensity [0, 1]

% downsample the pointcloud
skip = 200;
X = pt_location(1:skip:end,:);
y = pt_intensity(1:skip:end);

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
    'monitor', 100);

% Set initial parameter values:
% - this specification of the initial noise standard deviation is not
% necessary, but included here for illustration. If omitted, SPARSEBAYES
% will call SB2_PARAMETERSETTINGS itself to obtain an appropriate default
% for the noise (and other SETTINGS fields).
SETTINGS = SB2_ParameterSettings('NoiseStd',0.1);

% hyp trained by a GP
covfunc = {@covSEard}; hyp = []; hyp.cov = log([0.4480, 1.3557, 1.2241, 0.4005]);

BASIS = feval(covfunc{:}, hyp.cov, X);

% Run the main SPARSEBAYES function
[PARAMETER, HYPERPARAMETER, ~] = SparseBayes('Gaussian', BASIS, y, OPTIONS, SETTINGS);

w_infer	= PARAMETER.Value;

% Compute the inferred prediction function
test_BASIS = feval(covfunc{:}, hyp.cov, X(PARAMETER.Relevant, :), t)';
y_out = test_BASIS * w_infer;
% map the output into [0,1]
f = (y_out + abs(min(y_out))) / (max(y_out) - min(y_out));

time = toc(t0);

% output
out = [];
out.rvm.covfunc = covfunc;
out.rvm.hyp = hyp;
out.rvm.X = X(PARAMETER.Relevant, :);
out.parameters = PARAMETER;
out.hyperparameters = HYPERPARAMETER;
out.f = f;
% out.pc = newptcloud;
out.max_range = max_range;
out.time_second = time;
out.pcname = pcname;
out.pc_mean = pc_mean;
out.dim = size(PARAMETER.Relevant,1);

% plot the output
if nargin > 2
    if varargin{2} == true
        fsize = 20; % font size
        set(groot, 'defaultAxesTickLabelInterpreter','latex'); 
        set(groot, 'defaultLegendInterpreter','latex');  
        
        cmap = single(f .* repmat([255, 255, 255], length(f), 1)/ 255);
        newptcloud = pointCloud(t + pc_mean, 'Intensity', f, 'Color', cmap);

        figure; hold on
        pcshow(newptcloud)
        title('RVM Intensity Regression')
        axis equal, grid on
        set(gca,'fontsize',fsize)
        set(gca,'TickLabelInterpreter','latex')
        figuresize(21,21,'cm')

        orig_cmap = single(pt_intensity .* repmat([255, 255, 255], length(pt_intensity), 1)/ 255);
        orig_ptcloud = pointCloud(t + pc_mean, 'Intensity', pt_intensity, 'Color', orig_cmap);

        figure; hold on
        pcshow(orig_ptcloud)
        title('Original Point Cloud')
        axis equal, grid on
        set(gca,'fontsize',fsize)
        set(gca,'TickLabelInterpreter','latex')
        figuresize(21,21,'cm')
    end
end