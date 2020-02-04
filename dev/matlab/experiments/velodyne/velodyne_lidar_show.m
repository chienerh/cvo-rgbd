clc; clear; close all

file_folder = '../../../data/drive_0002_pcd/';
file_list = dir(strcat(file_folder, '*.pcd'));
file_number = length(file_list);

load('lidar_14-Dec-2018-15-31-16.mat');

target = pcread(file_list(1).name);
% target = pointCloud(pc_temp.Location, 'Color', repmat(uint8(pc_temp.Intensity),1,3), 'Intensity', pc_temp.Intensity);


tform = result{1};
ptCloudAligned = pctransform(target,tform);
% mergeSize = 0.01;
% ptCloudScene = pcmerge(target, ptCloudAligned, mergeSize);

% Store the transformation object that accumulates the transformation.
accumTform = tform; 

figure; hold on; grid on
hAxes = pcshow(ptCloudAligned, 'VerticalAxis','Y', 'VerticalAxisDir', 'Down');
title('Updated world scene')
% Set the axes property for faster rendering
hAxes.CameraViewAngleMode = 'auto';
hScatter = hAxes.Children;

k = 1;
j = 1; % last frame
for i = 2:file_number-1
    source = pcread(file_list(i).name);
%     source = pointCloud(pc_temp.Location, 'Color', repmat(uint8(pc_temp.Intensity),1,3), 'Intensity', pc_temp.Intensity);


    tform = result{k+1};
    k = k +1;
    
    % Transform the current point cloud to the reference coordinate system
    % defined by the first point cloud.
    accumTform = affine3d(tform.T * accumTform.T);
    ptCloudAligned = pctransform(source, accumTform);
    
%     pcshow(ptCloudAligned, 'VerticalAxis','Y', 'VerticalAxisDir', 'Down');
    pcshow(ptCloudAligned);
    
%     % Update the world scene.
%     ptCloudScene = pcmerge(ptCloudScene, ptCloudAligned, mergeSize);
% 
%     % Visualize the world scene.
%     hScatter.XData = ptCloudScene.Location(:,1);
%     hScatter.YData = ptCloudScene.Location(:,2);
%     hScatter.ZData = ptCloudScene.Location(:,3);
%     hScatter.CData = ptCloudScene.Color;
    drawnow('limitrate')
    
    disp('------------')
end
