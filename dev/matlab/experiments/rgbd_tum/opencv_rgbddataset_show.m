clc; clear; close all

file_folder = '../../../data/rgbd_dataset/freiburg3_nostructure_notexture_far/pcd/';
file_list = dir(strcat(file_folder, '*.pcd'));
file_number = length(file_list);

load('opencv_freiburg3_nostructure_notexture_far.mat');

tfinv = @(T) ([T(1:3,1:3)', -T(1:3,1:3)' * T(1:3,4); 0 0 0 1]);

target = pcread(file_list(1).name);

H0 = eye(4);

tform = affine3d(H0');
ptCloudAligned = pctransform(target,tform);
mergeSize = 0.001;
ptCloudScene = pcmerge(target, ptCloudAligned, mergeSize);

figure
hAxes = pcshow(ptCloudScene, 'VerticalAxis','Y', 'VerticalAxisDir', 'Down');
title('Updated world scene')
% Set the axes property for faster rendering
hAxes.CameraViewAngleMode = 'auto';
hScatter = hAxes.Children;

for i = 1:file_number
    source = pcread(file_list(i).name);
    
    R = reshape(cvrgbdposes(i,6:end),3,3)';
    t = cvrgbdposes(i,3:5)';
    H1 = [R, t; 0 0 0 1];
    H1 = tfinv(H1);
    H0 = H0 * H1;

    tform = affine3d(H0');
    
    % Transform the current point cloud to the reference coordinate system
    % defined by the first point cloud.
    ptCloudAligned = pctransform(source, tform);
    
    % Update the world scene.
    ptCloudScene = pcmerge(ptCloudScene, ptCloudAligned, mergeSize);

    % Visualize the world scene.
    hScatter.XData = ptCloudScene.Location(:,1);
    hScatter.YData = ptCloudScene.Location(:,2);
    hScatter.ZData = ptCloudScene.Location(:,3);
    hScatter.CData = ptCloudScene.Color;
    drawnow('limitrate')
    
    disp(['visualizing frame: ', num2str(i)])
end

% axis off
% figuresize(21,21,'cm')
% print -opengl -dpng -r300 fr1-xyz-1-250.png
% print -opengl -dpng -r600 fr1-xyz-1-250.png
