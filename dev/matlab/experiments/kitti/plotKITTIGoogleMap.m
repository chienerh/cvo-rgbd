clc; clear; close all

fsize = 20; % font size
set(groot, 'defaultAxesTickLabelInterpreter','latex'); 
set(groot, 'defaultLegendInterpreter','latex');

I = imread('kitti_05_bg.png');
I = rgb2gray(I);

hw_ratio = (size(I,1) / size(I,2));

width = [-295, 282];
w_offset = -5;
h_offset = 162;
height = hw_ratio * width;
% axlim = [width height];

figure;
imagesc(width + w_offset, height + h_offset, flipud(I)); 
colormap gray
hold on, axis equal tight
set(gca,'ydir','normal');
set(gca,'fontsize',fsize)
set(gca,'TickLabelInterpreter','latex')
set(gca,'XTick', -250:100:250);
set(gca,'YTick', -100:100:400);

Roffset = R2d(5 * pi/180); % map rotation offset

% ground truth trajectory
Hgt = load('gt05.txt');

% aling trajectory
p =  Roffset * [Hgt(:,4), Hgt(:,12)]';

% RKHS
load('kitti05_gicp_SE3_09-Sep-2018-11-32-22.mat');
init_frame = 1;

% get the transformations
Hrkhs = cell(size(result,1) - init_frame,1);
% Hrkhs{1} = [result{1}.T.R, result{1}.T.t; 0 0 0 1];
Hrkhs{1} = result{1}.T * [Ry(pi/2), zeros(3,1); 0 0 0 1];
p_rkhs = [];
p_rkhs(1,1) = 0;
p_rkhs(2,1) = 0;

for i = 2:size(result,1)
    if ~isempty(result{i})
%         A = [result{i}.T.R, result{i}.T.t; 0 0 0 1];
        A = result{i}.T;
        Hrkhs{i} = Hrkhs{i-1} * A;
        % x-y trajectory
        p_rkhs(1,i) = Hrkhs{i}(1,4);
        p_rkhs(2,i) = Hrkhs{i}(3,4);
    else
        j = i;
    end
end



% plot trajectory
% sicpC = [27,158,119]/255;
% gicpC = [217,95,2]/255;
% se3gicpC = 1.35*[117,112,179]/255;
% sicpnfC = [255,20,147]/255;

plot(p(1,init_frame:j), p(2,init_frame:j), '--y', 'linewidth', 3)
plot(p_rkhs(1,:) + p(1,init_frame), p_rkhs(2,:) + p(2,init_frame), '-', 'linewidth', 3)


legend('Ground truth', 'RKHS', 'location', 'northeast')

% set paper size
figuresize(21,21,'cm')

% print -painters -dpdf -r600 yourfile.pdf