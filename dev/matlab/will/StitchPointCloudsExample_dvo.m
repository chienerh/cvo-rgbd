clc; clear; close all
% tic;

%% Register Two Point Clouds
dataFile = fullfile(toolboxdir('vision'), 'visiondata', 'livingRoom.mat');
load(dataFile);

ptCloudRef = ptcloud_edge_filter(livingRoomData{1});
ptCloudCurrent = ptcloud_edge_filter(livingRoomData{2});

% ptCloudRef = livingRoomData{1};
% ptCloudCurrent = livingRoomData{2};

gridSize = 0.05;

% downsample point clouds using grids
fixed = [];
fixed.ptcloud = pcdownsample(ptCloudRef, 'gridAverage', gridSize);
% load RGB image
fixed.image = rgb2gray(livingRoomData{1}.Color);

% downsample point clouds using grids
moving = pcdownsample(ptCloudCurrent, 'gridAverage', gridSize);

% make rkhs registration object
tic
dvo = rgbd_dvo();
dvo.set_ptclouds(fixed, moving);
dvo.align();
tform = dvo.tform;
disp('RGBD DVO Object Transformation Estimate:')
disp(tform.T')
toc

ptCloudAligned = pctransform(livingRoomData{2},tform);

mergeSize = 0.015;
ptCloudScene = pcmerge(livingRoomData{1}, ptCloudAligned, mergeSize);

% Visualize the input images.
figure
subplot(2,2,1)
imshow(ptCloudRef.Color)
title('First input image')
drawnow

subplot(2,2,3)
imshow(ptCloudCurrent.Color)
title('Second input image')
drawnow

% Visualize the world scene.
subplot(2,2,[2,4])
pcshow(ptCloudScene, 'VerticalAxis','Y', 'VerticalAxisDir', 'Down')
title('Initial world scene')
xlabel('X (m)')
ylabel('Y (m)')
zlabel('Z (m)')
drawnow

%% Stitch a Sequence of Point Clouds
% To compose a larger 3-D scene, repeat the same procedure as above to
% process a sequence of point clouds. Use the first point cloud to
% establish the reference coordinate system. Transform each point cloud to
% the reference coordinate system. This transformation is a multiplication
% of pairwise transformations.

% Store the transformation object that accumulates the transformation.
% accumTform = tform; 
% 
% figure
% hAxes = pcshow(ptCloudScene, 'VerticalAxis','Y', 'VerticalAxisDir', 'Down');
% title('Updated world scene')
% % Set the axes property for faster rendering
% hAxes.CameraViewAngleMode = 'auto';
% hScatter = hAxes.Children;
% 
% for i = 3:length(livingRoomData)
%     ptCloudCurrent = ptcloud_edge_filter(livingRoomData{i});
%        
%     % Use previous moving point cloud as reference.
%     fixed.ptcloud = moving;
%     % load RGB image
%     fixed.image = rgb2gray(livingRoomData{i}.Color);
%     moving = pcdownsample(ptCloudCurrent, 'gridAverage', gridSize);
%     
% %     tform = pc_gradient_descent(moving, fixed);
%     dvo.set_ptclouds(fixed, moving);
%     dvo.align();
%     tform = dvo.tform;
% 
%     % Transform the current point cloud to the reference coordinate system
%     % defined by the first point cloud.
%     accumTform = affine3d(tform.T * accumTform.T);
%     ptCloudAligned = pctransform(livingRoomData{i}, accumTform);
%     
%     % Update the world scene.
%     ptCloudScene = pcmerge(ptCloudScene, ptCloudAligned, mergeSize);
% 
%     % Visualize the world scene.
%     hScatter.XData = ptCloudScene.Location(:,1);
%     hScatter.YData = ptCloudScene.Location(:,2);
%     hScatter.ZData = ptCloudScene.Location(:,3);
%     hScatter.CData = ptCloudScene.Color;
%     drawnow('limitrate')
% end
% 
% % During the recording, the Kinect was pointing downward. To visualize the
% % result more easily, let's transform the data so that the ground plane is
% % parallel to the X-Z plane.
% angle = -pi/10;
% A = [1,0,0,0;...
%      0, cos(angle), sin(angle), 0; ...
%      0, -sin(angle), cos(angle), 0; ...
%      0 0 0 1];
% ptCloudScene = pctransform(ptCloudScene, affine3d(A));
% pcshow(ptCloudScene, 'VerticalAxis','Y', 'VerticalAxisDir', 'Down', ...
%         'Parent', hAxes)
% title('Updated world scene')
% xlabel('X (m)')
% ylabel('Y (m)')
% zlabel('Z (m)')
% toc;