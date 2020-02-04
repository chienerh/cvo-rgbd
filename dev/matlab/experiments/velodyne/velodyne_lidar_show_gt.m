clc; clear; close all

% read all pcd directory
file_folder = '../../../data/2018-12-04/static_pcd/';
file_list = dir(strcat(file_folder, '*.pcd'));
file_number = length(file_list);

% load InEKF osometry data
load('inekf_pose_01.mat');

% get 4x4 tf from ROS position and quaterion (x,y,z,w). MATLAB uses
% w,x,y,z order.
ros2pose = @(t,q) ([quat2rotm(q), t; 0 0 0 1]);
tfinv = @(T) ([T(1:3,1:3)', -T(1:3,1:3)' * T(1:3,4); 0 0 0 1]);

% fixed tf from cassie pelvis (odom) to velodyne frame
tf_pelvis2velodyne = eye(4);
tf_pelvis2velodyne(1:3,4) = [0.065, 0.000, 0.530]';

pc_time = str2double(file_list(1).name(1:end-4));
target = pcread(file_list(1).name);
% target = pointCloud(pc_temp.Location, 'Color', repmat(uint8(pc_temp.Intensity),1,3), 'Intensity', pc_temp.Intensity);

% find the first matching pose
k = 1;
while (pc_time - pose(k,1)*1e-9) > 0
    k = k + 1;
end

init_tf = cell(file_number-1,1); % initial tf
odom = cell(file_number,1); % odometry data
% odom_time = pose(k,1) * 1e-9;   % time stamp
t = pose(k,2:4)';               % position
q = pose(k,[8,5,6,7]);          % orientation (quaternion)
% get 4x4 tf after applying change of basis
% odom{1} = ros2pose(t, q);
odom{1} = tfinv(tf_pelvis2velodyne) * ros2pose(t, q) * tf_pelvis2velodyne;
tform = affine3d(odom{1}');     % make an affine tf object using tf^transpose

% apply tf to the point cloud, map points from local sensor frame to the odom frame
ptCloudAligned = pctransform(target,tform);

% mergeSize = 0.01;
% ptCloudScene = pcmerge(target, ptCloudAligned, mergeSize);

figure; hold on; grid on
hAxes = pcshow(ptCloudAligned, 'VerticalAxis','Y', 'VerticalAxisDir', 'Down');
title('Updated world scene')
% Set the axes property for faster rendering
hAxes.CameraViewAngleMode = 'auto';
hScatter = hAxes.Children;

j = 1; % last frame
for i = 2:file_number
    pc_time = str2double(file_list(i).name(1:end-4));
    source = pcread(file_list(i).name);
%     source = pointCloud(pc_temp.Location, 'Color', repmat(uint8(pc_temp.Intensity),1,3), 'Intensity', pc_temp.Intensity);
    
    % find the next matching pose
    while (pc_time - pose(k,1)*1e-9) > 0
        k = k + 1;
    end
    
    t = pose(k,2:4)';               % position
    q = pose(k,[8,5,6,7]);          % orientation (quaternion)
    % get 4x4 tf after applying change of basis
%     odom{i} = ros2pose(t, q);
    odom{i} = tfinv(tf_pelvis2velodyne) * ros2pose(t, q) * tf_pelvis2velodyne;
    tform = affine3d(odom{i}');     % make an affine tf object using tf^transpose
    
    % fill in initial tf
    init_tf{i-1} = tfinv(odom{i-1}) * odom{i};
    
    % Transform the current point cloud to the reference coordinate system
    % defined by the first point cloud.
    ptCloudAligned = pctransform(source, tform);
    
    pcshow(ptCloudAligned, 'VerticalAxis','Y', 'VerticalAxisDir', 'Down');
    
%     % Update the world scene.
%     ptCloudScene = pcmerge(ptCloudScene, ptCloudAligned, mergeSize);
% 
%     % Visualize the world scene.
%     hScatter.XData = ptCloudScene.Location(:,1);
%     hScatter.YData = ptCloudScene.Location(:,2);
%     hScatter.ZData = ptCloudScene.Location(:,3);
%     hScatter.CData = ptCloudScene.Color;
    view(-45,45)
    drawnow('limitrate')
    
%     disp('------------')
end
