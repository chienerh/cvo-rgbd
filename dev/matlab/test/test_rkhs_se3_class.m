clc; clear; close all

%% Register Two Point Clouds

% use matlab dataset
% dataFile = fullfile(toolboxdir('vision'), 'visiondata', 'livingRoom.mat');
% load(dataFile);
% 
% target_id = 6;
% source_id = 7;

% ptRef_dense = livingRoomData{target_id};
% ptCurrent_dense = livingRoomData{source_id};
 
% ptCloudRef = ptcloud_edge_filter(ptRef_dense);
% ptCloudCurrent = ptcloud_edge_filter(ptCurrent_dense);

% save pointcloud for testing in cpp
% pcwrite(livingRoomData{target_id}, '/home/justin/research/rkhs_registration/data/matlab/fixed_dense.pcd','Encoding','ascii');
% pcwrite(livingRoomData{source_id}, '/home/justin/research/rkhs_registration/data/matlab/moving_dense.pcd','Encoding','ascii');

% gridSize = 0.1;
% fixed = pcdownsample(ptCloudRef, 'gridAverage', gridSize);
% moving = pcdownsample(ptCloudCurrent, 'gridAverage', gridSize);

% save pointcloud for testing in cpp
% pcwrite(fixed, '/home/justin/research/rkhs_registration/data/matlab/fixed.pcd','Encoding','ascii');
% pcwrite(moving, '/home/justin/research/rkhs_registration/data/matlab/moving.pcd','Encoding','ascii');

% use tum dataset
ptRef_dense = pcread('/home/justin/research/rkhs_registration/data/rgbd_dataset/freiburg1_desk/pcd_full/1305031453.359684.pcd');
ptCurrent_dense = pcread('/home/justin/research/rkhs_registration/data/rgbd_dataset/freiburg1_desk/pcd_full/1305031453.391690.pcd');
fixed = pcread('/home/justin/research/rkhs_registration/data/rgbd_dataset/freiburg1_desk/pcd_ds/1305031453.359684.pcd');
moving = pcread('/home/justin/research/rkhs_registration/data/rgbd_dataset/freiburg1_desk/pcd_ds/1305031453.391690.pcd');

% make rkhs registration object
rkhs_se3 = rkhs_se3_registration();
rkhs_se3.set_ptclouds(fixed, moving);
rkhs_se3.align();
tform_rkhs = rkhs_se3.tform;
disp('RKHS-SE(3) Object Transformation Estimate:')
disp(tform_rkhs.T')

% script version
tform = pc_gradient_descent(moving, fixed);
disp('RKHS-SE(3) Transformation Estimate:')
disp(tform.T')

% Visualize the output.
ptCloudAligned = pctransform(ptCurrent_dense,tform_rkhs);
mergeSize = 0.015;
ptCloudScene = pcmerge(ptRef_dense, ptCloudAligned, mergeSize);
figure
pcshow(ptCloudScene, 'VerticalAxis','Y', 'VerticalAxisDir', 'Down')
% title('Initial world scene')
xlabel('X (m)')
ylabel('Y (m)')
zlabel('Z (m)')


% Visualize the input images.
ptCloudAligned = pctransform(ptCurrent_dense,tform);
mergeSize = 0.015;
ptCloudScene = pcmerge(ptRef_dense, ptCloudAligned, mergeSize);
figure
pcshow(ptCloudScene, 'VerticalAxis','Y', 'VerticalAxisDir', 'Down')
% title('Initial world scene')
xlabel('X (m)')
ylabel('Y (m)')
zlabel('Z (m)')