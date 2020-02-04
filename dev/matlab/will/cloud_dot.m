% Computes the ''dot product'' of the clouds cloud_x and cloud_y
% The variable param = [sigma,ell,a^2,b^2]

function f = cloud_dot(cloud_x,cloud_y,param)

% Extract the parameters
sigma = param(1); ell = param(2);

% Extracting the lengths
length_x = size(cloud_x.Location,1);
length_y = size(cloud_y.Location,1);

% Construct our kernel
K = @(x,y)(sigma^2*exp(-vecnorm(x-y,2,2).^2/(2*ell^2)));

% The coefficient function
c_ij = @(i,j)(1e-5*double(cloud_x.Color(i,:))*double(cloud_y.Color(j,:))');
A_ij = @(i,j)(c_ij(i,j).*K(cloud_x.Location(i,:),cloud_y.Location(j,:))');

% get the coefficeints
AIJ = @(i)(A_ij(i,1:length_y)');
a_part = @(i)(AIJ(i));

% Calculate the function
a_i = zeros(length_x,1);
for i = 1:length_x
    k = i;
    
    partial_a = a_part(k);

    a_i(i) = sum(partial_a);
   
end

f = sum(a_i)';
