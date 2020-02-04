clc; clear; close all

file_folder = '../../../../data/rgbd_dataset/freiburg1_desk/pcd/';
file_list = dir(strcat(file_folder, '*.pcd'));
file_number = length(file_list);

load('freiburg1_desk_31-Jan-2019-03-30-14.mat');

% parameters
option = [];
option.max_range = 4;
option.min_range = 0.8;
% option.gridSize = 0.05;

target = pcread(file_list(1).name);
target = pcRangeFilter(target, option.max_range, option.min_range);

tform = result{1};
ptCloudAligned = pctransform(target,tform);
mergeSize = 0.001;
ptCloudScene = pcmerge(target, ptCloudAligned, mergeSize);

% Store the transformation object that accumulates the transformation.
accumTform = tform; 

% figure
% hAxes = pcshow(ptCloudScene, 'VerticalAxis','Y', 'VerticalAxisDir', 'Down');
% title('Updated world scene')
% % Set the axes property for faster rendering
% hAxes.CameraViewAngleMode = 'auto';
% hScatter = hAxes.Children;

k = 1;
j = 1; % last frame
for i = 2:2%file_number
    source = pcread(file_list(i).name);
    source = pcRangeFilter(source, option.max_range, option.min_range);
%     source = pcRangeFilter(source, 3);
    if isempty(source.Color)
       continue 
    end

    tform = result{k+1};
    k = k +1;
    
    % Transform the current point cloud to the reference coordinate system
    % defined by the first point cloud.
    accumTform = affine3d(tform.T * accumTform.T);
    ptCloudAligned = pctransform(source, accumTform);
    
    % Update the world scene.
    ptCloudScene = pcmerge(ptCloudScene, ptCloudAligned, mergeSize);

    % Visualize the world scene.
%     hScatter.XData = ptCloudScene.Location(:,1);
%     hScatter.YData = ptCloudScene.Location(:,2);
%     hScatter.ZData = ptCloudScene.Location(:,3);
%     hScatter.CData = ptCloudScene.Color;
%     drawnow('limitrate')
%     
%     disp(['visualizing frame: ', num2str(i)])
    disp(i)
end


fsize = 22;
figure; hold on; set(gca,'TickLabelInterpreter','latex', 'fontsize', fsize); 
hAxes = pcshow(ptCloudScene, 'VerticalAxis','Y', 'VerticalAxisDir', 'Down');
hAxes.CameraViewAngleMode = 'auto';
axis auto tight equal off
figuresize(21,21,'cm')
% print -opengl -dpng -r300 fr1-desk-1-2.png
