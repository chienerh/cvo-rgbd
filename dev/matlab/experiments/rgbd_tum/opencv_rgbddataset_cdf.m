clc; clear; close all

% file_folder = '../../../data/rgbd_dataset/freiburg1_desk/pcd/';
% file_list = dir(strcat(file_folder, '*.pcd'));
% file_number = length(file_list);

load('freiburg3_nostructure_notexture_far-gt-pose.mat');
load('opencv_freiburg3_nostructure_notexture_far.mat');

% get 4x4 tf from ROS position and quaterion (x,y,z,w). MATLAB uses
% w,x,y,z order.
ros2pose = @(t,q) ([quat2rotm(q), t; 0 0 0 1]);
tfinv = @(T) ([T(1:3,1:3)', -T(1:3,1:3)' * T(1:3,4); 0 0 0 1]);

% find the first matching pose
% k = 1;
% while (assoc(1) - gt_pose(k,1)) > 0 && k < size(gt_pose,1)
%     k = k + 1;
% end
k = 356;

% so(3) and R^3 distance metric
dso3 = @(R1,R2) (norm(logm(R1 * R2'), 'fro'));
dR3 = @(R1,R2,t1,t2) (norm(t1 - R1 * R2' * t2));

rot = 0; % rotation error
tran = 0; % translation error

t = gt_pose(k,2:4)';               % position
q = gt_pose(k,[8,5,6,7]);          % orientation (quaternion)
% get 4x4 tf after applying change of basis
T0 = ros2pose(t, q);

j = 1;
for i = 2:length(assoc)
    % find the next matching pose
    while (assoc(i) - gt_pose(k,1)) > 0 && k < size(gt_pose,1)
        k = k + 1;
    end
    
    t = gt_pose(k,2:4)';               % position
    q = gt_pose(k,[8,5,6,7]);          % orientation (quaternion)
    % get 4x4 tf 
    T1 = ros2pose(t, q);
    Trel = tfinv(T0) * T1;
    T0 = T1;
    
    % compute error
    H = [reshape(cvrgbdposes(i-1,6:end),3,3)', cvrgbdposes(i-1,3:5)'; 0 0 0 1];
    H = tfinv(H);
    if all(all(H == eye(4)))
        continue
    else
        rot(j,1) = dso3(H(1:3,1:3), Trel(1:3,1:3));
        tran(j,1) = dR3(H(1:3,1:3), Trel(1:3,1:3), H(1:3,4), Trel(1:3,4));
        j = j + 1;
    end
end

% CDF plots
figure; hold on; set(gca,'TickLabelInterpreter','latex', 'fontsize', 20); 
h = cdfplot(tran);
set(h, 'linewidth', 2.5, 'linestyle', '-')
xlabel('Position error (m)','Interpreter','latex')
ylabel('Fraction of data','Interpreter','latex')
title(''), grid on, axis tight
figuresize(21,12,'cm')
% print -opengl -dpng -r300 cvrgbd_cdf_position_error.png

figure; hold on; set(gca,'TickLabelInterpreter','latex', 'fontsize', 20); 
h = cdfplot(rot * 180/pi);
set(h, 'linewidth', 2.5, 'linestyle', '-')
xlabel('Orientation error (deg)','Interpreter','latex')
ylabel('Fraction of data','Interpreter','latex')
title(''), grid on, axis tight
figuresize(21,12,'cm')
% print -opengl -dpng -r300 cvrgbd_cdf_orientation_error.png

