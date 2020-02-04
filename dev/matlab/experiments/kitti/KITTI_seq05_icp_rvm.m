clc; clear; close all

file_folder = '../../../data/KITTI/sequence_5_xyzi/';
file_list = dir(strcat(file_folder, '*.pcd'));
file_number = length(file_list);

result = cell(file_number,1);
T = eye(4);
target = pcread(file_list(1).name);
% filter ranges greater than 20 meters
max_range = 10;
target = pcXYZIRangeFilter(target, max_range);
% downsample point clouds using grids
% gridStep = 0.2;
% target = pcdownsample(target,'gridAverage',gridStep);

idx = str2double(file_list(1).name(end-7:end-4));
skip = 3;
try
    for i = 1:skip:file_number-1     
        try
            source = pcread(file_list(i+skip).name);
        catch
            continue;
        end
        source = pcXYZIRangeFilter(source, max_range);
%         source = pcdownsample(source,'gridAverage',gridStep);
        
        idx = [idx; str2double(file_list(i+skip).name(end-7:end-4))];

        disp(['Registering: ', file_list(i).name(end-7:end-4), ' and ', file_list(i+skip).name(end-7:end-4)])
        t0 = tic;
        try
            T = gicp_robust_rvm_SE3(target, source);
        catch
            disp(['Failed registering: ', file_list(i).name(end-7:end-4), ' and ', file_list(i+skip).name(end-7:end-4)])
            T = eye(4);
        end
        result{i}.T = T;
        result{i}.time = toc(t0);
        disp('------------')
        target = source;
    end
    file_name = strcat('kitti05_gicp_robust_rvm_SE3_', datestr(now, 'dd-mmm-yyyy-HH-MM-SS'), '.mat');
    save(file_name, 'result', 'idx');
catch
    keyboard;
end