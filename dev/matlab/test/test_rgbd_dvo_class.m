clc; clear; close all

%% Register Two Point Clouds
dataset_name = 'freiburg1_desk2';
file_folder = ...
    strcat('../../../data/rgbd_dataset/', dataset_name, '/pcd/');
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

target = pcread(file_list(1).name);
% ptCloudRef = ptcloud_edge_filter(target);
ptCloudRef = pcRangeFilter(target, option.max_range, option.min_range);


% downsample point clouds using grids
fixed = [];
fixed.ptcloud = pcdownsample(ptCloudRef, 'gridAverage', option.gridSize);
% load RGB image
fixed.image = rgb2gray(imread(strcat(dataset_path, assoc(1,2))));

source = pcread(file_list(2).name);
ptCloudCurrent = pcRangeFilter(source, option.max_range, option.min_range);

% downsample point clouds using grids
moving = pcdownsample(ptCloudCurrent, 'gridAverage', option.gridSize);

% make rkhs registration object
tic
dvo = rgbd_dvo();
dvo.set_ptclouds(fixed, moving);
dvo.align();
tform_dvo = dvo.tform;
disp('RGBD DVO Object Transformation Estimate:')
disp(tform_dvo.T')
toc

% script version
tic
tform = pc_gradient_descent(moving, fixed.ptcloud);
disp('RKHS-SE(3) Transformation Estimate:')
disp(tform.T')
toc

% % Visualize the output.
% ptCloudAligned = pctransform(livingRoomData{source_id},tform_dvo);
% mergeSize = 0.015;
% ptCloudScene = pcmerge(livingRoomData{target_id}, ptCloudAligned, mergeSize);
% figure
% pcshow(ptCloudScene, 'VerticalAxis','Y', 'VerticalAxisDir', 'Down')
% % title('Initial world scene')
% xlabel('X (m)')
% ylabel('Y (m)')
% zlabel('Z (m)')
% 
% 
% % Visualize the input images.
% ptCloudAligned = pctransform(livingRoomData{source_id},tform);
% mergeSize = 0.015;
% ptCloudScene = pcmerge(livingRoomData{target_id}, ptCloudAligned, mergeSize);
% figure
% pcshow(ptCloudScene, 'VerticalAxis','Y', 'VerticalAxisDir', 'Down')
% % title('Initial world scene')
% xlabel('X (m)')
% ylabel('Y (m)')
% zlabel('Z (m)')