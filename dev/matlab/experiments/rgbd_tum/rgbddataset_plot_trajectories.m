clc; clear; close all

load('freiburg1_desk-gt-pose.mat');
load('freiburg1_desk_07-Jun-2019-17-44-45.mat');
load('opencv_freiburg1_desk.mat');

% load('freiburg3_structure_notexture_near-gt-pose.mat');
% load('freiburg3_structure_notexture_near_07-May-2019-02-18-54.mat');
% load('opencv_freiburg3_structure_notexture_near.mat');

% load('freiburg3_nostructure_notexture_far-gt-pose.mat');
% load('freiburg3_nostructure_notexture_far_07-May-2019-03-12-17');
% load('opencv_freiburg3_nostructure_notexture_far.mat');

% load('freiburg3_structure_notexture_far-gt-pose.mat');
% load('freiburg3_structure_notexture_far_07-May-2019-03-03-43');
% load('opencv_freiburg3_structure_notexture_far.mat');


% get 4x4 tf from ROS position and quaterion (x,y,z,w). MATLAB uses
% w,x,y,z order.
ros2pose = @(t,q) ([quat2rotm(q), t; 0 0 0 1]);
tfinv = @(T) ([T(1:3,1:3)', -T(1:3,1:3)' * T(1:3,4); 0 0 0 1]);

% find the first matching pose
k = 1;
while (assoc(1) - gt_pose(k,1)) > 0 && k < size(gt_pose,1)
    k = k + 1;
end
% k = 356;

t = gt_pose(k,2:4)';               % position
q = gt_pose(k,[8,5,6,7]);          % orientation (quaternion)
% get 4x4 tf after applying change of basis
T0 = ros2pose(t, q);

% ground truth trajectory
position_gt = [];
position_gt.x = gt_pose(k:end,2);
position_gt.y = gt_pose(k:end,3);
position_gt.z = gt_pose(k:end,4);

% rkhs-se(3)
H = T0; % initial tf
position_rkhs = [];
position_rkhs.x(1) = H(1,4);
position_rkhs.y(1) = H(2,4);
position_rkhs.z(1) = H(3,4);
for i = 2:size(result,1)
    if isa(result{i}, 'affine3d')
        H = H * result{i}.T';
    end
    position_rkhs.x(i,1) = H(1,4);
    position_rkhs.y(i,1) = H(2,4);
    position_rkhs.z(i,1) = H(3,4);
end

% opencv rgbd vo
H = T0; % initial tf
position_cv = [];
position_cv.x(1) = H(1,4);
position_cv.y(1) = H(2,4);
position_cv.z(1) = H(3,4);
for i = 1:size(cvrgbdposes,1)
    R = reshape(cvrgbdposes(i,6:end),3,3)';
    t = cvrgbdposes(i,3:5)';
    H1 = [R, t; 0 0 0 1];
    H1 = tfinv(H1);
    H = H * H1;
    position_cv.x(i+1,1) = H(1,4);
    position_cv.y(i+1,1) = H(2,4);
    position_cv.z(i+1,1) = H(3,4);
end

% colors
green = [0.2980 .6 0];
crimson = [220,20,60]/255; 
darkblue = [0 .2 .4];
Darkgrey = [.25 .25 .25];
darkgrey = [.35 .35 .35];
lightgrey = [.7 .7 .7];
Lightgrey = [.9 .9 .9];
VermillionRed = [156,31,46]/255;
DupontGray = [144,131,118]/255;
Azure = [53, 112, 188]/255;
purple = [178, 102, 255]/255;
orange = [255,110,0]/255;


fsize = 22;
figure; hold on; set(gca,'TickLabelInterpreter','latex', 'fontsize', fsize); 
% ground truth
h{1} = plot3(position_gt.x, position_gt.y, position_gt.z);
set(h{1}, 'linewidth', 3.5, 'linestyle', '-.', 'color', [darkblue, .75])
% initial position
h{2} = plot3(position_gt.x(1), position_gt.y(1), position_gt.z(1), 's');
set(h{2}, 'MarkerSize', 14, 'MarkerFaceColor', VermillionRed,'MarkerEdgeColor', VermillionRed)
% opencv rgbd vo
h{3} = plot3(position_cv.x, position_cv.y, position_cv.z);
set(h{3}, 'linewidth', 3, 'linestyle', '--', 'color', [orange, .75])
% rkhs-se(3)
h{4} = plot3(position_rkhs.x, position_rkhs.y, position_rkhs.z);
set(h{4}, 'linewidth', 3, 'linestyle', '-', 'color', [green, .75])

legend([h{1}, h{3}, h{4}, h{2}], 'Ground truth', ...
    'OpenCV RGB-D VO', 'RGB-D CVO', 'Initial position', 'location', 'best','Interpreter','latex')
% xlabel('x (m)','Interpreter','latex')
% ylabel('y (m)','Interpreter','latex')
% zlabel('z (m)','Interpreter','latex')
% title('fr3-structure\_notexture\_far', 'FontWeight', 'normal', 'fontsize', fsize), grid on, axis auto equal
% title('fr3-structure\_notexture\_far', 'FontWeight', 'normal', 'fontsize', fsize), grid on, axis auto equal
% title('fr3-structure\_notexture\_far', 'FontWeight', 'normal', 'fontsize', fsize), grid on, axis auto equal
title('fr1-desk', 'FontWeight', 'normal', 'fontsize', fsize), grid on, axis auto equal
figuresize(21,21,'cm')
print -opengl -dpng -r300 tum_top_fr1-desk.png
