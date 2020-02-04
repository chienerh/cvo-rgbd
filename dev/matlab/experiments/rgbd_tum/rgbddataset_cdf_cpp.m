clc; clear; close all

% load('freiburg3_nostructure_notexture_far-gt-pose.mat');
% load('freiburg3_nostructure_notexture_far_07-May-2019-03-12-17');
% load('opencv_freiburg3_nostructure_notexture_far.mat');

% load('freiburg3_nostructure_texture_far-gt-pose.mat');
% load('freiburg3_nostructure_texture_far_08-May-2019-00-45-35.mat');
% load('opencv_freiburg3_nostructure_texture_far.mat');

% load('freiburg3_structure_notexture_far-gt-pose.mat');
% result = load('freiburg3_structure_notexture_far_30-May-2019_1.csv');
% load('opencv_freiburg3_structure_notexture_far.mat');

% load('freiburg3_structure_notexture_near-gt-pose.mat');
% load('freiburg3_structure_notexture_near_07-May-2019-02-18-54.mat');
% load('opencv_freiburg3_structure_notexture_near.mat');

% load('freiburg3_structure_texture_far-gt-pose.mat');
% % load('freiburg3_structure_texture_far_04-Jan-2019-07-46-14.mat');
% load('freiburg3_structure_texture_far_31-Jan-2019-17-22-02.mat');
% load('opencv_freiburg3_structure_texture_far.mat');

% load('freiburg3_structure_texture_near-gt-pose.mat');
% load('freiburg3_structure_texture_near_07-May-2019-22-25-11.mat');
% load('opencv_freiburg3_structure_texture_near.mat');

load('freiburg1_desk-gt-pose.mat');
result = load('cvo_poses.csv');
% load('freiburg1_desk_06-Jun-2019-16-39-39.mat');
load('opencv_freiburg1_desk.mat');

% load('freiburg1_desk2-gt-pose.mat');
% load('freiburg1_desk2_07-May-2019-10-43-05.mat')
% load('opencv_freiburg1_desk2.mat');

% load('freiburg1_room-gt-pose.mat');
% load('freiburg1_room_07-May-2019-14-12-32.mat');
% load('opencv_freiburg1_room.mat');

%TODO
% load('freiburg3_long_office_household-gt-pose.mat');
% load('freiburg3_long_office_household_07-May-2019-19-39-43.mat');
% load('opencv_freiburg3_long_office_household.mat');

%TODO
% load('freiburg2_pioneer_slam-gt-pose.mat');
% load('freiburg2_pioneer_slam_07-May-2019-18-18-35.mat');
% load('opencv_freiburg2_pioneer_slam.mat');

%TODO
% load('freiburg2_pioneer_360-gt-pose.mat');
% load('freiburg2_pioneer_360_07-Jun-2019-14-41-11.mat');
% load('opencv_freiburg2_pioneer_360.mat');

% load('freiburg2_desk-gt-pose.mat');
% load('freiburg2_desk_07-May-2019-23-19-57.mat');
% load('opencv_freiburg2_desk.mat');


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

% so(3) and R^3 distance metric
dso3 = @(R1,R2) (norm(logm(R1 * R2'), 'fro'));
dR3 = @(R1,R2,t1,t2) (norm(t1 - R1 * R2' * t2));

rot = 0; % rotation error
tran = 0; % translation error
rot_cv = 0; % rotation error
tran_cv = 0; % translation error

t = gt_pose(k,2:4)';               % position
q = gt_pose(k,[8,5,6,7]);          % orientation (quaternion)
% get 4x4 tf after applying change of basis
T0 = ros2pose(t, q);

j = 1; % pose estimates iterator
for i = 2:length(assoc)
    % find the next matching pose
    while (assoc(i) - gt_pose(k,1)) > 0 && k < size(gt_pose,1)
        k = k + 1;
    end
    
%     check if the timestapms are close
    if abs(assoc(i) - gt_pose(k,1)) > 0.01
%         rot(j,1) = nan;
%         tran(j,1) = nan;
%         rot_cv(j,1) = nan;
%         tran_cv(j,1) = nan;
%         j = j + 1;
%         continue;
        disp('true')
        disp(assoc(i) - gt_pose(k,1))
    end
    
    t = gt_pose(k,2:4)';               % position
    q = gt_pose(k,[8,5,6,7]);          % orientation (quaternion)
    % get 4x4 tf 
    T1 = ros2pose(t, q);
    Trel = tfinv(T0) * T1;
    T0 = T1;
    
    % compute rkhs error
%     if isa(result{i}, 'affine3d')
%         H = result{i}.T';
% %         H = [reshape(result(i-1,6:end),3,3)', result(i-1,3:5)'; 0 0 0 1];
%         rot(j,1) = dso3(H(1:3,1:3), Trel(1:3,1:3));
%         tran(j,1) = dR3(H(1:3,1:3), Trel(1:3,1:3), H(1:3,4), Trel(1:3,4));
%     else
%         rot(j,1) = nan;
%         tran(j,1) = nan;
%     end
    H = [reshape(result(i-1,6:end),3,3)', result(i-1,3:5)'; 0 0 0 1];
%     H = tfinv(H);
    if ~all(all(H == eye(4)))
        rot(j,1) = dso3(H(1:3,1:3), Trel(1:3,1:3));
        tran(j,1) = dR3(H(1:3,1:3), Trel(1:3,1:3), H(1:3,4), Trel(1:3,4));
    else
        rot(j,1) = nan;
        tran(j,1) = nan;
    end
    
    % compute opencv benchmark error
    H = [reshape(cvrgbdposes(i-1,6:end),3,3)', cvrgbdposes(i-1,3:5)'; 0 0 0 1];
    H = tfinv(H);
    if ~all(all(H == eye(4)))
        rot_cv(j,1) = dso3(H(1:3,1:3), Trel(1:3,1:3));
        tran_cv(j,1) = dR3(H(1:3,1:3), Trel(1:3,1:3), H(1:3,4), Trel(1:3,4));
    else
        rot_cv(j,1) = nan;
        tran_cv(j,1) = nan;
    end
    j = j + 1;
end

% CDF plots
fsize = 22;
figure; hold on; set(gca,'TickLabelInterpreter','latex', 'fontsize', fsize); 
h{1} = cdfplot(tran); 
set(h{1}, 'linewidth', 2.5, 'linestyle', '-')
h{2} = cdfplot(tran_cv);
set(h{2}, 'linewidth', 2.5, 'linestyle', '--')
legend([h{1}, h{2}], 'RGB-D CVO', 'OpenCV RGB-D VO', 'location', 'best','Interpreter','latex')
xlabel('Position error (m)','Interpreter','latex')
ylabel('Fraction of data','Interpreter','latex')
title('fr1-desk', 'FontWeight', 'normal', 'fontsize', fsize), grid on, axis tight
figuresize(21,12,'cm')
print -opengl -dpng -r300 tum_cdf_fr1-desk_grad_position_error_50_10000_0.03.png

figure; hold on; set(gca,'TickLabelInterpreter','latex', 'fontsize', fsize); 
h{1} = cdfplot(rot * 180/pi); 
set(h{1}, 'linewidth', 2.5, 'linestyle', '-')
h{2} = cdfplot(rot_cv * 180/pi);
set(h{2}, 'linewidth', 2.5, 'linestyle', '--')
legend([h{1}, h{2}], 'RGB-D CVO', 'OpenCV RGB-D VO', 'location', 'best','Interpreter','latex')
xlabel('Orientation error (deg)','Interpreter','latex')
ylabel('Fraction of data','Interpreter','latex')
title(''), grid on, axis tight
figuresize(21,12,'cm')
print -opengl -dpng -r300 tum_cdf_fr1-desk_grad_orientation_error_50_10000_0.03.png

