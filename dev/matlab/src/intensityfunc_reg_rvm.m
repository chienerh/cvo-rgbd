function out = intensityfunc_reg_rvm(ptCloud, varargin)

pt_location = squeeze(ptCloud.Location); % x y z coordinates

% intensity [0, 1]
if max(ptCloud.Intensity) > 1
    pt_intensity = ptCloud.Intensity / 255; 
else
    pt_intensity = ptCloud.Intensity;
end

% downsample the pointcloud
skip = 1;
X = double(pt_location(1:skip:end,:));
y = double(pt_intensity(1:skip:end));

% Test points are all points
t = double(pt_location);

% measuring regression time
t0 = tic;

% RVM

iterations = 1500;

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
% covfunc = {@covSEard}; hyp = []; hyp.cov = log([0.4480, 1.3557, 1.2241, 0.4005]);
covfunc = {@covSEiso}; hyp = []; hyp.cov = log([0.05, 0.5]);

BASIS = feval(covfunc{:}, hyp.cov, X);
BASIS(BASIS < 1e-4) = 0;
BASIS = sparse(BASIS);
BASIS = 0.5 * (BASIS + BASIS');

% Run the main SPARSEBAYES function
[PARAMETER, HYPERPARAMETER, ~] = SparseBayes('Gaussian', BASIS, sparse(y), OPTIONS, SETTINGS);
% [PARAMETER, HYPERPARAMETER, ~] = SparseBayes('Gaussian', BASIS, y, OPTIONS, SETTINGS);

w_infer	= PARAMETER.Value;

% Compute the inferred prediction function
test_BASIS = feval(covfunc{:}, hyp.cov, X(PARAMETER.Relevant, :), t)';
y_out = test_BASIS * w_infer;
% map the output into [0,1]
% f = (y_out + abs(min(y_out))) / (max(y_out) - min(y_out));
f = (y_out + abs(min(y_out)));
f = f ./ max(f);

time = toc(t0);

% output
out = [];
out.rvm.covfunc = covfunc;
out.rvm.hyp = hyp;
out.rvm.X = X(PARAMETER.Relevant, :);
PARAMETER.Value = full(PARAMETER.Value);
out.parameters = PARAMETER;
HYPERPARAMETER.Alpha = full(HYPERPARAMETER.Alpha);
out.hyperparameters = HYPERPARAMETER;
out.f = f;
% out.pc = newptcloud;
% out.max_range = max_range;
out.time_second = time;
% out.pcname = pcname;
out.dim = size(PARAMETER.Relevant,1);

% plot the output
if nargin > 1
    if varargin{1} == true
        fsize = 20; % font size
        set(groot, 'defaultAxesTickLabelInterpreter','latex'); 
        set(groot, 'defaultLegendInterpreter','latex');  
        
        cmap = single(f .* repmat([255, 255, 255], length(f), 1)/ 255);
        newptcloud = pointCloud(t, 'Intensity', f, 'Color', cmap);

        figure; hold on
        pcshow(newptcloud)
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
    end
end