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
b = 32;        % block size for point selection
thres_shift = 7;

% load association file, i.e., matched RGB and Depth timestamps
% dataset_name = "freiburg3_structure_notexture_far";5
dataset_name = 'freiburg1_desk';
dataset_path = ...
    strcat('/home/justin/research/rkhs_registration/data/rgbd_dataset/', ...
    dataset_name, '/');
assoc_filename = strcat(dataset_path, 'assoc.txt');
assoc = import_assoc_file(assoc_filename);

folder = 'pcd_ds/';
% create point clouds
% for i = 1:size(assoc,1)
for i = 310:328
     strTime = assoc(i,1);
     rgb_name = assoc(i,2);
     depth_name = assoc(i,4);
     
     % create point selector class
     pointSelector = select_points();
     pointSelector.load_img(dataset_path, rgb_name, depth_name);
     pointSelector.calculate_threshold();
     pointSelector.smoothen_threshold();
     num_points = pointSelector.select();
%      num_points = pointSelector.make_map(1);
     
     % visualize filtered img
%      selected_rgb = pointSelector.rgb;
%      rgb_map = zeros(480,640,3);
%      rgb_map(:,:,1) = pointSelector.map;
%      rgb_map(:,:,2) = pointSelector.map;
%      rgb_map(:,:,3) = pointSelector.map;
%      selected_rgb(~rgb_map)=0;
%      figure(2);
%      imshow(selected_rgb);
%      
     pointSelector.generate_pcd(fx, fy, cx, cy, dataset_path, folder, strTime)
     
end