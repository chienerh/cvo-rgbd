clc; clear; close all


dataset_name = 'freiburg2_pioneer_360';
file_folder = ...
    strcat('../../../../data/rgbd_dataset/', dataset_name, '/pcd_ds/');
file_list = dir(strcat(file_folder, '*.pcd'));
file_number = length(file_list);

% parameters
option = [];
option.max_range = 4;
option.min_range = 0.8;
option.gridSize = 0.05;

result = cell(file_number,1);

target = pcread(file_list(1).name);
% ptCloudRef = ptcloud_edge_filter(target);
ptCloudRef = pcRangeFilter(target, option.max_range, option.min_range);

% downsample point clouds using grids
fixed = pcdownsample(ptCloudRef, 'gridAverage', option.gridSize);

tform = affine3d;
result{1} = tform;

registration_time = zeros(file_number-1,1);
for i = 2:file_number
    try
        source = pcread(file_list(i).name);
%         ptCloudCurrent = ptcloud_edge_filter(source);
        ptCloudCurrent = pcRangeFilter(source, option.max_range, option.min_range);

        % downsample point clouds using grids
        moving = pcdownsample(ptCloudCurrent, 'gridAverage', option.gridSize);

        disp(['Registering: ', file_list(i-1).name, ' and ', file_list(i).name])
        disp(['Registering: ', num2str(i-1), ' and ', num2str(i)])

        t0 = tic;
        tform = pc_gradient_descent(moving, fixed, tform.T');
        registration_time(i-1) = toc(t0);
        disp(['Registration time in seconds: ', num2str(registration_time(i-1))])

        fixed = moving;
        result{i} = tform;
    catch
        disp(['Registration failed at between frames: ', num2str(i-1), ' and ', num2str(i)])
        registration_time(i-1) = nan;
        fixed = moving;
        result{i} = nan;
    end
    disp('------------')
end

file_name = strcat(dataset_name, '_', datestr(now, 'dd-mmm-yyyy-HH-MM-SS'), '.mat');
save(file_name, 'result', 'option', 'dataset_name', 'registration_time');
