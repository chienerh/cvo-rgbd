clc; clear; close all

file_folder = '../../../data/KITTI/seq00/';
file_list = dir(strcat(file_folder, '*.pcd'));
file_number = length(file_list);

result = cell(file_number,1);

try
    for i = 1:file_number
        try
            target = pcread(file_list(i).name);
            out = intensityfunc_reg_rvm(target);
            out.name = file_list(i).name;
            result{i} = out;
        catch
            continue;
        end
        if mod(i,100) == 0
            disp(i)
        elseif mod(i,500) == 0
            file_name = strcat('kitti00_rvm_reg_', datestr(now, 'dd-mmm-yyyy-HH-MM-SS'), '.mat');
            save(file_name, 'result');
        end            
    end
    file_name = strcat('kitti00_rvm_reg_', datestr(now, 'dd-mmm-yyyy-HH-MM-SS'), '.mat');
    save(file_name, 'result');
catch
    keyboard;
end