classdef select_points < handle
    properties
        b = 32;                 % block size for different threshold
        ths_shift = 7;          % threshold offset from average intensity in each block
        ths_factor = 1;         % threshold factor for modifying threhold in recursive function: ths = ths_factor*ths
        ths_step = 0.2;         % threshold increment/decrement for recursive function
        ds_box_size = 8;        % size of downsample box within each block, can be view as a max pool kernel size
        sigma = 1;              % sigma used for gaussian kernel
        num_wanted = 2500;      % desired number of point
        num_tolerance = 0.15;   % tolerance for final number of points
        scaling_factor = 5000;  % scaling factor for depth image
        rgb;                    % rgb image
        depth;                  % depth image
        intensity;              % intensity image
        i_dx;                   % intensity dx
        i_dy;                   % intensity dy
        abs_squared_grad;       % magnitude of gradient, ie. sqrt(dx^2+dy^2)
        h;                      % height of the rgb image
        w;                      % width of the rgb image
        hb;                     % height / block size
        wb;                     % width / block size
        ths;                    % a matrix to store thresholds for each block
        ths_smoothed;           % a matrix for smoothened threholds
        map;                    % a mapping for selected points, 1 for selected, 0 for discarded
        num_selected = 0;       % number of points that are selected
    end
    
    methods(Static)
        
    end
    
    methods
        function initialize(obj)
           obj.h = size(obj.rgb,1);
           obj.w = size(obj.rgb,2);
           obj.hb = obj.h/obj.b;
           obj.wb = obj.w/obj.b;
           obj.ths = zeros(obj.hb,obj.wb);
           obj.map = false(obj.h,obj.w);
           obj.num_selected = 0;
        end
        
        function load_img(obj,dataset_path, rgb_name, depth_name)
            
           % load rgb image
           obj.rgb = imread(char(strcat(dataset_path, rgb_name)));
           obj.intensity = rgb2gray(obj.rgb);
           
           % they seem to use absolute squared gradient in DSO (no sqrt)
%            [obj.i_dx, obj.i_dy] = imgradientxy(obj.intensity);
%            obj.abs_squared_grad = obj.i_dx.^2 + obj.i_dy.^2;    % gradient magnitude

           % apply gaussian filter before calculating gradient
%            obj.intensity = imgaussfilt(obj.intensity,obj.sigma);
           
           obj.variance_of_laplacian();

           % calculate gradient magnitude: = sqrt(dx^2+dy^2)
           obj.abs_squared_grad = imgradient(obj.intensity);
           
           % apply another gaussian filter on gradient to remove noise
%            obj.abs_squared_grad = imgaussfilt(obj.abs_squared_grad,obj.sigma);
           
           % median filter doesn't work well
%            obj.abs_squared_grad = medfilt2(obj.abs_squared_grad,[obj.sigma,obj.sigma]);

           % entropy filter is slow and doesn't work well
           % if you use entropy filter, you'll need to change ths_shift
%            obj.abs_squared_grad = entropyfilt(obj.intensity);

           % load depth image
           obj.depth = single(imread(char(strcat(dataset_path, depth_name))));
           obj.depth(obj.depth == 0) = nan;
           
           % initialize parameters
           obj.initialize();
            
           % visualization for debugging
           figure(1);
           imshow(obj.abs_squared_grad/max(max(obj.abs_squared_grad))*1);
%            figure(6);
%            imshow(obj.intensity);
%            figure(4);
%            imshow(obj.i_dx);
%            figure(5);
%            imshow(obj.i_dy);
        end
        
        function variance_of_laplacian(obj)
            kernel = [0 1 0; 1 -4 1; 0 1 0];
%             I_lap = locallapfilt(obj.intensity,0.4,1);
            
            figure(7);
            imshow(I_lap);
        end
        
        function calculate_threshold(obj)
            for y = 1 : obj.hb
              for x = 1 : obj.wb
                  % extract blocks with size b*b
                  cur_grad = obj.abs_squared_grad((y-1)*obj.b+1:y*obj.b,(x-1)*obj.b+1:x*obj.b);     % extract 32*y to 32*x
                  % find median of gradient magnitudes within this block
                  obj.ths(y,x) = median(reshape(cur_grad,[1,obj.b*obj.b]),2) + obj.ths_shift;
        
              end
            end
        end
        
        function smoothen_threshold(obj)
            % smoothen the threshold using a 3 by 3 kernel and convolution
            % to get average of them
            avg_kernel = 1/9*ones(3);
            obj.ths_smoothed = conv2(obj.ths,avg_kernel,'same');
%             obj.ths_smoothed = obj.ths;
            
            for i = 2:obj.wb-1
               obj.ths_smoothed(1,i) =  obj.ths_smoothed(1,i)*9/6;
               obj.ths_smoothed(obj.hb,i) =  obj.ths_smoothed(obj.hb,i)*9/6;
            end
            for j = 2:obj.hb-1
               obj.ths_smoothed(j,1) =  obj.ths_smoothed(j,1)*9/6;
               obj.ths_smoothed(j,obj.wb) =  obj.ths_smoothed(j,obj.wb)*9/6;
            end
            obj.ths_smoothed(1,1) =  obj.ths_smoothed(1,1)*9/4;
            obj.ths_smoothed(1,obj.wb) =  obj.ths_smoothed(1,obj.wb)*9/4;
            obj.ths_smoothed(obj.hb,1) =  obj.ths_smoothed(obj.hb,1)*9/4;
            obj.ths_smoothed(obj.hb,obj.wb) =  obj.ths_smoothed(obj.hb,obj.wb)*9/4;

%             figure(7);
%             h1 = heatmap(obj.ths);
%             figure(8);
%             h2 = heatmap(obj.ths_smoothed);
           
        end
        
        function n = select(obj)
             n = 0;
             d = obj.ds_box_size;
             stride = obj.b/d;
             
             % loop through blocks
             for y = 1 : obj.hb
                  for x = 1 : obj.wb
               
                      % extract blocks with size b*b
                      cur_grad = obj.abs_squared_grad((y-1)*obj.b+1:y*obj.b,(x-1)*obj.b+1:x*obj.b);
                      cur_map = false(obj.b,obj.b);
                      cur_ths = obj.ths_smoothed(y,x)*obj.ths_factor;
%                       cur_ths = obj.ths(y,x)*obj.ths_factor;
                      
                      % select the max value within each ds box, say a 8*8 little box inside the 32*32 block
                      % obj.b is the size of the big block, 32; d is the size of the little box, 8
                      % lets loop through the little boxes in each block
                      for di = 1:stride
                         for dj = 1:stride
                             best_val = -1;
                             best_idx = [-1,-1];
                             % lets go into the little box and find the max val
                             for i=1:d
                                 for j=1:d
                                     % extract gradient value of current pixel
                                     cur_gradij = cur_grad((di-1)*d+i,(dj-1)*d+j);
                                     % if grad value is higher than threshold
                                     if cur_gradij > cur_ths
                                        if cur_gradij > best_val
                                            best_val = cur_gradij;
                                            best_idx(1) =  (di-1)*d+i;  % extract idx, matlab starts from 1 so 
                                            best_idx(2) = (dj-1)*d+j;   % we need to do this conversion
                                        end
                                     end
                                 end
                             end
                             
                             % if we find the max val within the box
                             if best_val > 0
%                                 obj.map((y-1)*obj.b+best_idx(1),(x-1)*obj.b+best_idx(2)) = 1;
                                
                                % update locally
                                cur_map(best_idx(1),best_idx(2)) = 1;                              
                                obj.num_selected = obj.num_selected + 1;
                                n = n + 1;
                             end
                         end
                      end
                      
                      % update this 32 block map to the global map
                      obj.map((y-1)*obj.b+1:y*obj.b,(x-1)*obj.b+1:x*obj.b) = cur_map;
                  end
             end
             
             % below are just for visualization
%              selected_rgb = obj.rgb;
%              rgb_map = zeros(480,640,3);
%              rgb_map(:,:,1) = obj.map;
%              rgb_map(:,:,2) = obj.map;
%              rgb_map(:,:,3) = obj.map;
%              selected_rgb(~rgb_map)=0;
%              figure(2);
%              imshow(selected_rgb);
        end
        
        function n = remove(obj)
          
        end
        
        function n = make_map(obj,mode)
            
            % select or remove points
            % mode1: select, mode2: remove
%             if mode == 1
%                 s = obj.select();
%             elseif mode == 2
%                 s = obj.select();
%             end

            s = obj.select();
            
            ratio = obj.num_selected / obj.num_wanted;
            % if we pick up too many points
            if ratio > 1+obj.num_tolerance
                obj.ths_factor = obj.ths_factor + obj.ths_step;
                obj.num_selected = 0;
                n = obj.make_map(2);
            % if the points are still not enough
            elseif ratio < 1-obj.num_tolerance
                obj.ths_factor = obj.ths_factor - obj.ths_step;
                obj.num_selected = 0;
                n = obj.make_map(1);
%                 disp("increasing...");
            % if we have the number we want
            else
                n = obj.num_selected;
            end
        end
        
        function generate_pcd(obj, fx, fy, cx, cy, dataset_path, folder,strTime)
            
            points = single(obj.rgb);
            U = repmat(0:size(obj.depth,2)-1, size(obj.depth,1), 1);
            V = repmat([0:size(obj.depth,1)-1]', 1, size(obj.depth,2));
            points(:,:,3) = obj.depth / obj.scaling_factor;
            points(:,:,1) = (U - cx) .* points(:,:,3) ./ fx;
            points(:,:,2) = (V - cy) .* points(:,:,3) ./ fy;
            points(~obj.map) = NaN;
            point_cloud = pointCloud(points, 'Color', obj.rgb);   
            point_cloud = removeInvalidPoints(point_cloud);
            
%             figure(3);
%             pcshow(point_cloud);
            
            % write point clouds
%             path_to_save = char(strcat(dataset_path, folder, strTime, '.pcd'));
%             pcwrite(point_cloud, path_to_save,'Encoding','ascii');
        end
               
    end
end