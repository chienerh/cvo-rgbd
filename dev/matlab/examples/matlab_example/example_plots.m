% example plots
clc; clear all; close all

fsize = 20; % font size
set(groot, 'defaultAxesTickLabelInterpreter','latex'); 
set(groot, 'defaultLegendInterpreter','latex');

dataFile = fullfile(toolboxdir('vision'), 'visiondata', 'livingRoom.mat');
load(dataFile);

% Extract two consecutive point clouds and use the first point cloud as
% reference.
ptCloudRef = livingRoomData{1};
ptCloudCurrent = livingRoomData{2};

figure
hAxes = pcshow(ptCloudRef, 'VerticalAxis','Y', 'VerticalAxisDir', 'Down');
title('First (Fixed) Point Cloud', 'Interpreter','latex')
hAxes.CameraViewAngleMode = 'auto';
axis equal tight, grid on
set(gca,'fontsize',fsize)
set(gca,'TickLabelInterpreter','latex')
figuresize(21,21,'cm')
print -opengl -dpng -r300 first_pc.png


figure
hAxes = pcshow(ptCloudCurrent, 'VerticalAxis','Y', 'VerticalAxisDir', 'Down');
title('Second (Moving) Point Cloud', 'Interpreter','latex')
hAxes.CameraViewAngleMode = 'auto';
axis equal tight, grid on
set(gca,'fontsize',fsize)
set(gca,'TickLabelInterpreter','latex')
figuresize(21,21,'cm')
print -opengl -dpng -r300 second_pc.png

% Registered
gridSize = 0.1;
fixed = pcdownsample(ptCloudRef, 'gridAverage', gridSize);
moving = pcdownsample(ptCloudCurrent, 'gridAverage', gridSize);

T = gicp_SE3(fixed, moving);
tform = affine3d(T');
ptCloudAligned = pctransform(ptCloudCurrent,tform);

mergeSize = 0.015;
ptCloudScene = pcmerge(ptCloudRef, ptCloudAligned, mergeSize);

figure
hAxes = pcshow(ptCloudScene, 'VerticalAxis','Y', 'VerticalAxisDir', 'Down');
title('Registered Point Cloud', 'Interpreter','latex')
hAxes.CameraViewAngleMode = 'auto';
axis equal tight, grid on
set(gca,'fontsize',fsize)
set(gca,'TickLabelInterpreter','latex')
figuresize(21,21,'cm')
print -opengl -dpng -r300 registered_pc.png
