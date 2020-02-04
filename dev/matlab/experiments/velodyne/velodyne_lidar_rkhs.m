clc; clear; close all

file_folder = '../../../data/drive_0002_pcd/';
file_list = dir(strcat(file_folder, '*.pcd'));
file_number = length(file_list);

% load('odom_tf_01.mat');

result = cell(file_number,1);

pc_temp = pcread(file_list(1).name);
% remove zero intensity points
idx = (pc_temp.Intensity == 0) | (pc_temp.Location(:,1) < 0);
ptCloudRef = pointCloud(pc_temp.Location(~idx,:), 'Color', repmat(uint8(255*pc_temp.Intensity(~idx,:)*sqrt(1/3)),1,3), 'Intensity', pc_temp.Intensity(~idx,:));

% downsample point clouds using grids
gridSize = 0.25;
fixed = pcdownsample(ptCloudRef, 'gridAverage', gridSize);

tform = affine3d;
% ptCloudAligned = pctransform(ptCloudRef,tform);
% mergeSize = 0.01;
% ptCloudScene = pcmerge(ptCloudRef, ptCloudAligned, mergeSize);
result{1} = tform;

% Store the transformation object that accumulates the transformation.
% accumTform = tform; 

% figure
% hAxes = pcshow(ptCloudScene, 'VerticalAxis','Y', 'VerticalAxisDir', 'Down');
% title('Updated world scene')
% % Set the axes property for faster rendering
% hAxes.CameraViewAngleMode = 'auto';
% hScatter = hAxes.Children;

k = 1;
j = 1; % last frame
for i = 2:file_number-1
    pc_temp = pcread(file_list(i).name);
    % remove zero intensity points
    idx = (pc_temp.Intensity == 0) | (pc_temp.Location(:,1) < 0);
    ptCloudCurrent = pointCloud(pc_temp.Location(~idx,:), 'Color', repmat(uint8(255*pc_temp.Intensity(~idx,:)*sqrt(1/3)),1,3), 'Intensity', pc_temp.Intensity(~idx,:));
    try
        moving = pcdownsample(ptCloudCurrent, 'gridAverage', gridSize);
    catch
        disp(['Frame ', num2str(i), ' skipped.'])
        continue
    end
    
    disp(['Registering: ', file_list(i-1).name, ' and ', file_list(i).name])
    disp(['Registering: ', num2str(j), ' and ', num2str(i)])
    
    tic;
%     tform = pc_gradient_descent(moving, fixed, init_tf{i-1});
    tform = pc_gradient_descent(moving, fixed);
    toc;
    fixed = moving;
    result{k+1} = tform;
    k = k + 1;
    j = i;
    
    % Transform the current point cloud to the reference coordinate system
    % defined by the first point cloud.
%     accumTform = affine3d(tform.T * accumTform.T);
%     ptCloudAligned = pctransform(ptCloudCurrent, accumTform);
    
    % Update the world scene.
%     ptCloudScene = pcmerge(ptCloudScene, ptCloudAligned, mergeSize);

    % Visualize the world scene.
%     hScatter.XData = ptCloudScene.Location(:,1);
%     hScatter.YData = ptCloudScene.Location(:,2);
%     hScatter.ZData = ptCloudScene.Location(:,3);
%     hScatter.CData = ptCloudScene.Color;
%     drawnow('limitrate')
    
    disp('------------')
end

file_name = strcat('lidar_', datestr(now, 'dd-mmm-yyyy-HH-MM-SS'), '.mat');
save(file_name, 'result', 'gridSize');
