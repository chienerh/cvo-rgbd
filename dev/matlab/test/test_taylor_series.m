% Script that tests the quality of using a Taylor approximation
%% Register Two Point Clouds
tic;
dataFile = fullfile(toolboxdir('vision'), 'visiondata', 'livingRoom.mat');
load(dataFile);

% Extract two consecutive point clouds and use the first point cloud as
% reference.
ptCloudRef = livingRoomData{1};
ptCloudCurrent = livingRoomData{2};

%% Adjusting the format
gridSize = 0.1;
fixed = pcdownsample(ptCloudRef, 'gridAverage', gridSize);
moving = pcdownsample(ptCloudCurrent, 'gridAverage', gridSize);

% Note that the downsampling step does not only speed up the registration,
% but can also improve the accuracy.

%% Defining the paramaters
sigma = 0.1;
ell   = 0.15;
c     = 7;
d     = 7;
param = [sigma,ell,c,d];

%% Important background functions
% The 'hat' function, R^3\to\Skew_3
hat = @(omega)([0,-omega(3),omega(2);...
                omega(3),0,-omega(1);...
                -omega(2),omega(1),0]);

% Defining the group action
% h = @(R,T,p)(R*p+T);
% h_inv = @(R,T,p)(R'*p-R'*T);
h = @(R,T)([R,T;0,0,0,1]);
h_inv = @(R,T)([R',-R'*T;0,0,0,1]');

%% Determine omega, v, and step
[omega,v,step,coef] = transform_identity_NR_4_vectorized(fixed,moving,param);
%%
% The trajectory planning
T = linspace(0,2*step,100);
disp(coef(5));

% Draw the picture
figure; hold on;
% The 4^th-order approximation
guess_fcn4 = @(t)(coef(1)*t.^4+coef(2)*t.^3+coef(3)*t.^2+coef(4)*t+coef(5)); %
plot(T,guess_fcn4(T),'r','LineWidth',2);
% The 2^nd-order approximation
guess_fcn2 = @(t)(coef(3)*t.^2+coef(4)*t+coef(5)); %
plot(T,guess_fcn2(T),'k','LineWidth',2);
% Determining the exact values
t_coarse = linspace(0,2*step,10);
dot_exact = t_coarse;
for i = 1:length(t_coarse)
    dt = t_coarse(i);
    th = norm(omega); homega = hat(omega);
    dR = eye(3) + (sin(dt*th)/th)*homega + ((1-cos(dt*th))/th^2)*homega^2;
    dT = (dt*eye(3) + (1-cos(dt*th))/(th^2)*homega + ((dt*th-sin(dt*th))/th^3)*homega^2)*v;
    TRANSFORM = h_inv(dR,dT); % Our current transformation
    tform = affine3d(TRANSFORM);
    moved = pctransform(moving,tform);
    dot_exact(i) = cloud_dot(fixed,moved,param);
end
plot(t_coarse,dot_exact,'--gs',...
    'LineWidth',2,...
    'MarkerSize',10,...
    'MarkerEdgeColor','b',...
    'MarkerFaceColor',[0.5,0.5,0.5]);
grid;
toc;