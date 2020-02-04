function label = rgb2labelKITTI(image)
% rgb to label conversion

% class_name = [r, g, b, label];
building = [128, 0, 0, 1];
vegetation = [128, 128, 0, 2];
car = [64, 0, 128, 3];
road = [128, 64, 128, 4];
fence = [64, 192, 0, 5];
sidewalk = [0, 0, 192, 6];
pole = [0, 64, 64, 7];

% Segmentation results use different color for two classes
wall = [64, 64, 128, 5];
post = [192, 192, 128, 7];

% Only for sequence 05
% signage = [192, 128, 128, 8];
% sky = [128, 128, 128, 9];

w = size(image, 1);
h = size(image, 2);
label = zeros(w, h);

for i = 1 : w
    for j = 1 : h
        rgb = image(i, j, :);
        if rgb(1) == building(1) && rgb(2) == building(2) && rgb(3) == building(3)
            label(i, j) = building(4);
        elseif rgb(1) == vegetation(1) && rgb(2) == vegetation(2) && rgb(3) == vegetation(3)
            label(i, j) = vegetation(4);
        elseif rgb(1) == car(1) && rgb(2) == car(2) && rgb(3) == car(3)
            label(i, j) = car(4);
        elseif rgb(1) == road(1) && rgb(2) == road(2) && rgb(3) == road(3)
            label(i, j) = road(4);
        elseif rgb(1) == fence(1) && rgb(2) == fence(2) && rgb(3) == fence(3)
            label(i, j) = fence(4);
        elseif rgb(1) == sidewalk(1) && rgb(2) == sidewalk(2) && rgb(3) == sidewalk(3)
            label(i, j) = sidewalk(4);
        elseif rgb(1) == pole(1) && rgb(2) == pole(2) && rgb(3) == pole(3)
            label(i, j) = pole(4);
        elseif rgb(1) == wall(1) && rgb(2) == wall(2) && rgb(3) == wall(3)
            label(i, j) = wall(4);
        elseif rgb(1) == post(1) && rgb(2) == post(2) && rgb(3) == post(3)
            label(i, j) = post(4);
%         elseif rgb(1) == signage(1) && rgb(2) == signage(2) && rgb(3) == signage(3)
%             label(i, j) = signage(4);
%         elseif rgb(1) == sky(1) && rgb(2) == sky(2) && rgb(3) == sky(3)
%             label(i, j) = sky(4);
        end
    end
end
label = uint8(label);

end
