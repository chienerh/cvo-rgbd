clc; clear; close all

dataFile = fullfile(toolboxdir('vision'), 'visiondata', 'livingRoom.mat');
load(dataFile);

% Extract two consecutive point clouds and use the first point cloud as
% reference.
ptCloudRef = livingRoomData{1};
ptCloudCurrent = livingRoomData{2};

gridSize = 0.2;
fixed = pcdownsample(ptCloudRef, 'gridAverage', gridSize);
fixed.Intensity = 0.299 * double(fixed.Color(:,1)) + 0.587 * double(fixed.Color(:,2)) + 0.114 * double(fixed.Color(:,3));
moving = pcdownsample(ptCloudCurrent, 'gridAverage', gridSize);
moving.Intensity = 0.299 * double(moving.Color(:,1)) + 0.587 * double(moving.Color(:,2)) + 0.114 * double(moving.Color(:,3));

% Note that the downsampling step does not only speed up the registration,
% but can also improve the accuracy.

% tform = pcregrigid(moving, fixed, 'Metric','pointToPlane','Extrapolate', true);
T = RKHS_registration_SE3_gp(fixed, moving); % no range filtering, no downsampling
tform = affine3d(T');
ptCloudAligned = pctransform(ptCloudCurrent,tform);

mergeSize = 0.015;
ptCloudScene = pcmerge(ptCloudRef, ptCloudAligned, mergeSize);

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

%-------------------------------------------------------------------------
%{
% Store the transformation object that accumulates the transformation.
accumTform = tform;

figure
hAxes = pcshow(ptCloudScene, 'VerticalAxis','Y', 'VerticalAxisDir', 'Down');
title('Updated world scene')
% Set the axes property for faster rendering
hAxes.CameraViewAngleMode = 'auto';
hScatter = hAxes.Children;

for i = 3:length(livingRoomData)
    ptCloudCurrent = livingRoomData{i};

    % Use previous moving point cloud as reference.
    fixed = moving;
    moving = pcdownsample(ptCloudCurrent, 'gridAverage', gridSize);
    moving.Intensity = 0.299 * double(moving.Color(:,1)) + 0.587 * double(moving.Color(:,2)) + 0.114 * double(moving.Color(:,3));

    % Apply ICP registration.
%     tform = pcregrigid(moving, fixed, 'Metric','pointToPlane','Extrapolate', true);
    T = RKHS_registration_SE3_rvm(fixed, moving); % no range filtering, no downsampling
    tform = affine3d(T');

    % Transform the current point cloud to the reference coordinate system
    % defined by the first point cloud.
    accumTform = affine3d(tform.T * accumTform.T);
    ptCloudAligned = pctransform(ptCloudCurrent, accumTform);

    % Update the world scene.
    ptCloudScene = pcmerge(ptCloudScene, ptCloudAligned, mergeSize);

    % Visualize the world scene.
    hScatter.XData = ptCloudScene.Location(:,1);
    hScatter.YData = ptCloudScene.Location(:,2);
    hScatter.ZData = ptCloudScene.Location(:,3);
    hScatter.CData = ptCloudScene.Color;
    drawnow('limitrate')
end

% During the recording, the Kinect was pointing downward. To visualize the
% result more easily, let's transform the data so that the ground plane is
% parallel to the X-Z plane.
angle = -pi/10;
A = [1,0,0,0;...
     0, cos(angle), sin(angle), 0; ...
     0, -sin(angle), cos(angle), 0; ...
     0 0 0 1];
ptCloudScene = pctransform(ptCloudScene, affine3d(A));
pcshow(ptCloudScene, 'VerticalAxis','Y', 'VerticalAxisDir', 'Down', ...
        'Parent', hAxes)
title('Updated world scene')
xlabel('X (m)') 
ylabel('Y (m)')
zlabel('Z (m)')
%}
