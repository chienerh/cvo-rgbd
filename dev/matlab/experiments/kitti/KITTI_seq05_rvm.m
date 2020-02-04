clc; clear; close all

file_folder = '../../../data/sequence_5_xyzi/';
file_list = dir(strcat(file_folder, '*.pcd'));
file_number = length(file_list);

result = cell(file_number,1);

try
    for i = 1:file_number-1

        target = file_list(i).name;
        source = file_list(i+1).name;

        disp(['Registering: ', num2str(i), ' and ', num2str(i+1)])
        t0 = tic;
%         result{i}.T = RKHS_registration_SE3_rvm(target, source);
        try
            tform = pcregrigid(pcread(source), pcread(target), 'Metric','pointToPlane');
        catch
            disp(['Failed registering: ', num2str(i), ' and ', num2str(i+1)])
            tform.T = eye(4);
        end
        result{i}.T = tform.T';
        result{i}.time = toc(t0);
        disp('------------')
    end
    file_name = strcat('kitti05_rkhs_rvm_', datestr(now, 'dd-mmm-yyyy-HH-MM-SS'), '.mat');
    save(file_name, 'result');
catch
    keyboard;
end