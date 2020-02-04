clc; clear; close all


dataset_name = 'freiburg1_desk2';
file_folder = ...
    strcat('../../../../data/rgbd_dataset/', dataset_name, '/pcd/');
file_list = dir(strcat(file_folder, '*.pcd'));
file_number = length(file_list);
dataset_path = ...
    strcat('/home/maani/workspace/perl/rkhs_registration/data/rgbd_dataset/', ...
    dataset_name, '/');
assoc_filename = strcat(dataset_path, 'assoc.txt');
assoc = import_assoc_file(assoc_filename);

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
fixed = [];
fixed.ptcloud = pcdownsample(ptCloudRef, 'gridAverage', option.gridSize);
% load RGB image
fixed.image = rgb2gray(imread(strcat(dataset_path, assoc(1,2))));

tform = affine3d;
result{1} = tform;
dvo = rgbd_dvo();

registration_time = zeros(file_number-1,1);
for i = 2:50%file_number
    try
        source = pcread(file_list(i).name);
%         ptCloudCurrent = ptcloud_edge_filter(source);
        ptCloudCurrent = pcRangeFilter(source, option.max_range, option.min_range);

        % downsample point clouds using grids
        moving = pcdownsample(ptCloudCurrent, 'gridAverage', option.gridSize);

        disp(['Registering: ', file_list(i-1).name, ' and ', file_list(i).name])
        disp(['Registering: ', num2str(i-1), ' and ', num2str(i)])

        t0 = tic;
        dvo.set_ptclouds(fixed, moving);
        dvo.align();
        tform = dvo.tform;
        registration_time(i-1) = toc(t0);
        disp(['Registration time in seconds: ', num2str(registration_time(i-1))])

        fixed.ptcloud = moving;
%         load RGB image
        fixed.image = rgb2gray(imread(strcat(dataset_path, assoc(i,2))));
        result{i} = tform;
    catch
        keyboard;
        disp(['Registration failed at between frames: ', num2str(i-1), ' and ', num2str(i)])
        registration_time(i-1) = nan;
        fixed = moving;
        result{i} = nan;
    end
    disp('------------')
end

file_name = strcat(dataset_name, '_dvo_', datestr(now, 'dd-mmm-yyyy-HH-MM-SS'), '.mat');
save(file_name, 'result', 'option', 'dataset_name', 'registration_time');
