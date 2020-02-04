%{
- The color images are stored as 640×480 8-bit RGB images in PNG format.
- The depth maps are stored as 640×480 16-bit monochrome images in PNG format.
- The color and depth images are already pre-registered using the OpenNI 
driver from PrimeSense, i.e., the pixels in the color and depth images 
correspond already 1:1.
- The depth images are scaled by a factor of 5000, i.e., a pixel value 
of 5000 in the depth image corresponds to a distance of 1 meter from the 
camera, 10000 to 2 meter distance, etc. A pixel value of 0 means missing 
value/no data.


The depth images in our datasets are reprojected into the frame of the color 
camera, which means that there is a 1:1 correspondence between pixels in the 
depth map and the color image.
 
The conversion from the 2D images to 3D point clouds works as follows. Note 
that the focal lengths (fx/fy), the optical center (cx/cy), the distortion 
parameters (d0-d4) and the depth correction factor are different for each 
camera. The Python code below illustrates how the 3D point can be computed 
from the pixel coordinates and the depth value:

fx = 525.0  # focal length x
fy = 525.0  # focal length y
cx = 319.5  # optical center x
cy = 239.5  # optical center y

factor = 5000 # for the 16-bit PNG files
# OR: factor = 1 # for the 32-bit float images in the ROS bag files

for v in range(depth_image.height):
  for u in range(depth_image.width):
    Z = depth_image[v,u] / factor;
    X = (u - cx) * Z / fx;
    Y = (v - cy) * Z / fy;

Camera          fx      fy      cx      cy      d0      d1      d2      d3      d4
(ROS default)	525.0	525.0	319.5	239.5	0.0     0.0     0.0     0.0     0.0
Freiburg 1 RGB	517.3	516.5	318.6	255.3	0.2624	-0.9531	-0.0054	0.0026	1.1633
Freiburg 2 RGB	520.9	521.0	325.1	249.7	0.2312	-0.7849	-0.0033	-0.0001	0.9172
Freiburg 3 RGB	535.4	539.2	320.1	247.6	0       0       0       0       0

For more information see: https://vision.in.tum.de/data/datasets/rgbd-dataset/file_formats

Author: Maani Ghaffari Jadidi
Date: 26-12-2018
%}

clc; clear; close all

% RGB intrinsic calibration parameters
% Freiburg 1
fx = 517.3;  % focal length x
fy = 516.5;  % focal length y
cx = 318.6;  % optical center x
cy = 255.3;  % optical center y

% Freiburg 2
% fx = 520.9;  % focal length x
% fy = 521.0;  % focal length y
% cx = 325.1;  % optical center x
% cy = 249.7;  % optical center y

% Freiburg 3
% fx = 535.4;  % focal length x
% fy = 539.2;  % focal length y
% cx = 320.1;  % optical center x
% cy = 247.6;  % optical center y

scaling_factor = 5000;  % depth scaling factor

% set this flag true if want edge extracted point clouds instead of full
% extract_edge = true;
extract_edge = false;

% set this flag true is want to apply gradient on images
apply_gradient = true;
% apply_gradient = false;

if extract_edge == true && apply_gradient == true
   error("Please do not enable extract_edge and apply_gradient at the same time!"); 
end

% parameters used for gradient
intensity_thres = 50;
depth_thres = 0.2;

% set this flag true if want to downsample
down_sample = true;
% down_sample = false;

% parameters used for downsampling
max_range = 4;
min_range = 0.8;
gridSize = 0.03;
gridSizeTemp = gridSize;

% load association file, i.e., matched RGB and Depth timestamps
% dataset_name = "freiburg3_structure_notexture_far";
dataset_name = 'freiburg1_desk';
dataset_path = ...
    strcat('/home/justin/research/rkhs_registration/data/rgbd_dataset/', ...
    dataset_name, '/');
assoc_filename = strcat(dataset_path, 'assoc.txt');
assoc = import_assoc_file(assoc_filename);

folder = 'pcd_full/';
% create point clouds
% for i = 1:size(assoc,1)
for i = 450:450

   gridSizeTemp = gridSize;
   
   pc_name = assoc(i,1); % using RGB image timestamp
   % load RGB image
   rgb = imread(char(strcat(dataset_path, assoc(i,2))));
   
   % load Depth image
   depth = single(imread(char(strcat(dataset_path, assoc(i,4)))));
   depth(depth == 0) = nan;
   
   % compute points xyz
   points = single(rgb);
   U = repmat(0:size(depth,2)-1, size(depth,1), 1);
   V = repmat([0:size(depth,1)-1]', 1, size(depth,2));
   points(:,:,3) = depth / scaling_factor;
   points(:,:,1) = (U - cx) .* points(:,:,3) ./ fx;
   points(:,:,2) = (V - cy) .* points(:,:,3) ./ fy;
   
   
   point_cloud = pointCloud(points, 'Color', rgb);   
   
%    f1 = figure(1);
%    pcshow(point_cloud);
   
   if extract_edge
       point_cloud = ptcloud_edge_filter(point_cloud); 
   
   elseif apply_gradient
       point_cloud_filtered = copy(point_cloud);
       inten_thres_temp = intensity_thres;
       depth_thres_temp = depth_thres;
       
%        while point_cloud_filtered.Count > 2000
           gray = rgb2gray(rgb);
           [gray_dx, gray_dy] = imgradientxy(gray);
%            vec_gray_dx = abs(reshape(gray_dx,[1,size(gray_dx,1)*size(gray_dx,2)]));
%            f4 = figure(4);
%            plot([1:size(vec_gray_dx,2)],vec_gray_dx);
%            hist(vec_gray_dx,0:900);
           
%            b = sum(vec_gray_dx > 200);
%            a = mean(abs(vec_gray_dx));
%            inten_thres_temp = a+250;
           [depth_dx, depth_dy] = imgradientxy(depth/scaling_factor);
           idx = or(or(or(abs(gray_dx) > inten_thres_temp,abs(gray_dy) > inten_thres_temp),...
                  abs(depth_dx)>depth_thres_temp),abs(depth_dy)>depth_thres_temp);
           point_cloud_filtered = ptcloud_gradient_filter(point_cloud,idx);
       
%            inten_thres_temp = inten_thres_temp + 20;
           
%        end
       point_cloud = copy(point_cloud_filtered);
   end
   
%    f2 = figure(2);
%    pcshow(point_cloud);
   
   if down_sample
%        if point_cloud.Count > 2000
           
%            frac = 2000/point_cloud.Count;
%            point_cloud = pcRangeFilter(point_cloud, max_range, min_range);
           point_cloud_ds = pcdownsample(point_cloud, 'gridAverage', gridSize);
%            point_cloud_ds = pcdownsample(point_cloud, 'random', frac);
           folder = 'pcd_ds/'; 
%            while point_cloud_ds.Count < 2000
%               gridSizeTemp = gridSizeTemp - 0.002;
%               if gridSizeTemp < 0
%                   break;
%               end
%               point_cloud_ds = pcdownsample(point_cloud, 'gridAverage', gridSizeTemp);
% 
%            end
           point_cloud = copy(point_cloud_ds);
%        end
   end
%    f3 = figure(3);
%    pcshow(point_cloud);
   
   % write point clouds
   path_to_save = char(strcat(dataset_path, folder, assoc(i,1), '.pcd'));
   pcwrite(point_cloud, path_to_save,'Encoding','ascii');
end