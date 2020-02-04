function out = intensityfunc_reg_gp_centralize(pcname, varargin)

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

% output
out = [];
out.gp.covfunc = covfunc;
out.gp.likfunc = likfunc;
out.gp.inffunc = @infGaussLik;
out.gp.hyp = hyp;
out.gp.X = X;
out.m = m;
out.s2 = s2;
out.f = f;
out.post = post;
% out.pc = newptcloud;
out.max_range = max_range;
out.time_second = time;
out.pcname = pcname;
out.pc_mean = pc_mean;
out.dim = size(m,1);

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
        title('GP Intensity Regression')
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