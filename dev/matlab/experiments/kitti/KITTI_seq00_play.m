clc; clear; close all

file_folder = '../../../data/KITTI/seq00/';
file_list = dir(strcat(file_folder, '*.pcd'));
file_number = length(file_list);

result = cell(file_number,1);
T = eye(4);
target = pcread(file_list(1).name);
% filter ranges greater than 20 meters
% max_range = 8;
% target = pcXYZIRangeFilter(target, max_range);

figure; hold on
h = pcshow(target); grid on
figuresize(21,21,'cm')


for i = 1:file_number-2
    try
        target = pcread(file_list(i+2).name);
        cla(h);
        h = pcshow(target); grid on
        pause(.1)
    catch
        continue;
    end
end