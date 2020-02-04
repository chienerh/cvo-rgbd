clc; clear; close all

tic;

%% Register Two Point Clouds
% target
pc_temp = pcread('data/2018-12-04/static_pcd/1543936635.293857000.pcd');
ptCloudRef = pointCloud(pc_temp.Location, 'Color', repmat(uint8(pc_temp.Intensity),1,3));

% source
pc_temp = pcread('data/2018-12-04/static_pcd/1543936635.400312000.pcd');
ptCloudCurrent = pointCloud(pc_temp.Location, 'Color', repmat(uint8(pc_temp.Intensity),1,3));
clear pc_temp

gridSize = 0.05;
fixed = pcdownsample(ptCloudRef, 'gridAverage', gridSize);
moving = pcdownsample(ptCloudCurrent, 'gridAverage', gridSize);

tform = pc_gradient_descent(moving, fixed);
ptCloudAligned = pctransform(ptCloudCurrent,tform);

mergeSize = 0.015;
ptCloudScene = pcmerge(ptCloudRef, ptCloudAligned, mergeSize);

% Store the transformation object that accumulates the transformation.
accumTform = tform; 

figure
hAxes = pcshow(ptCloudScene, 'VerticalAxis','Y', 'VerticalAxisDir', 'Down');
title('Updated world scene')
% Set the axes property for faster rendering
hAxes.CameraViewAngleMode = 'auto';
hScatter = hAxes.Children;

% for i = 3:length(livingRoomData)
%     ptCloudCurrent = ptcloud_edge_filter(livingRoomData{i});
%        
%     % Use previous moving point cloud as reference.
%     fixed = moving;
%     moving = pcdownsample(ptCloudCurrent, 'gridAverage', gridSize);
%     
%     % Apply ICP registration.
%     %tform = pcregistericp(moving, fixed, 'Metric','pointToPlane','Extrapolate', true);
%     % <<Our new algorithm>>
%     tform = pc_gradient_descent(moving, fixed);
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

% During the recording, the Kinect was pointing downward. To visualize the
% result more easily, let's transform the data so that the ground plane is
% parallel to the X-Z plane.
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
toc;