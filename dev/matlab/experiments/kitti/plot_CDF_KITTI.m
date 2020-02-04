clc; clear; close all

fsize = 20; % font size
set(groot, 'defaultAxesTickLabelInterpreter','latex'); 
set(groot, 'defaultLegendInterpreter','latex');

% GICP
load('kitti05_gicp_SE3_09-Sep-2018-11-32-22.mat');
% init_frame = 1000;
% idx(idx==103) = [];
result(2:2:end) = [];
idx = idx + 1;

% ground truth trajectory
Hgt = load('gt05.txt');
Hgt = Hgt(idx,:);
A = [];
for i = 1:length(idx)-1
    H1 = [Hgt(i,1:4); Hgt(i,5:8); Hgt(i,9:12); 0 0 0 1];
    H2 = [Hgt(i+1,1:4); Hgt(i+1,5:8); Hgt(i+1,9:12); 0 0 0 1];
    A{i,1} = H1 \ H2;
    A{i,1}(1:3,4) = A{i,1}(1:3,4) * 1e-3;
end

egicp = [];
egicp.R =[];
egicp.t = [];
time = [];
for i = 1:size(result,1)-1
    e = logm(A{i} \ result{i}.T);
    eR = e(1:3,1:3);
    et = e(1:3,4);
    egicp.R(i,1) = norm(eR(:));
    egicp.t(i,1) = norm(et(:));
    time(i,1) = result{i}.time;
end

% GICP
load('kitti05_gicp_SE3_09-Sep-2018-11-32-22.mat');
% idx(idx==103) = [];
% result(103) = [];
result(2:2:end) = [];
idx = idx + 1;

ervm = [];
ervm.R =[];
ervm.t = [];
for i = 1:size(result,1)-1
    e = logm(A{i} \ result{i}.T);
    eR = e(1:3,1:3);
    et = e(1:3,4);
    ervm.R(i,1) = norm(eR(:));
    ervm.t(i,1) = norm(et(:));
    time(i,2) = result{i}.time;
end


h = [];
figure; hold on
axis tight
h(:,1) = cdfplot(egicp.R * 180/pi);
set( h(:,1), 'LineWidth', 2, 'LineStyle', '--');

h(:,2) = cdfplot(ervm.R * 180/pi);
set( h(:,2), 'LineWidth', 2, 'LineStyle', '-');
legend('GICP-SE(3)', '$\mathcal{H}_k$-GICP-SE(3)', 'location', 'best')
set(gca,'fontsize',fsize)
set(gca,'TickLabelInterpreter','latex')
xlabel('Rotation Error (deg)', 'Interpreter','latex')
ylabel('Fraction of Data', 'Interpreter','latex')
title('')

% set paper size
figuresize(21,14,'cm')
print -opengl -dpng -r300 kitti00_gicp_rvm_rot.png

h = [];
figure; hold on
axis tight
h(:,1) = cdfplot(egicp.t);
set( h(:,1), 'LineWidth', 2, 'LineStyle', '--');

h(:,2) = cdfplot(ervm.t);
set( h(:,2), 'LineWidth', 2, 'LineStyle', '-');
legend('GICP-SE(3)', '$\mathcal{H}_k$-GICP-SE(3)', 'location', 'best')
set(gca,'fontsize',fsize)
set(gca,'TickLabelInterpreter','latex')
xlabel('Translation Error (m)', 'Interpreter','latex')
ylabel('Fraction of Data', 'Interpreter','latex')
title('')

% set paper size
figuresize(21,14,'cm')
print -opengl -dpng -r300 kitti00_gicp_rvm_tra.png

% plot time
h = [];
figure; hold on
axis tight
h(:,1) = cdfplot(time(:,1));
set( h(:,1), 'LineWidth', 2, 'LineStyle', '--');
h(:,2) = cdfplot(time(:,2));
set( h(:,2), 'LineWidth', 2, 'LineStyle', '-');
legend('GICP-SE(3)', '$\mathcal{H}_k$-GICP-SE(3)', 'location', 'best')
set(gca,'fontsize',fsize)
set(gca,'TickLabelInterpreter','latex')
xlabel('Time (sec)', 'Interpreter','latex')
ylabel('Fraction of Data', 'Interpreter','latex')
title('')
% set paper size
figuresize(21,14,'cm')
print -opengl -dpng -r300 kitti00_time_gicp_rvm.png