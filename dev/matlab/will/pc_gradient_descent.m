function tform = pc_gradient_descent(moving, fixed, varargin)

%% Defining the paramaters
param = [];
param.sigma = 0.1;
param.ell   = 0.15;
param.a2    = 7;
param.b2    = 7;
% The maximum number of iterations of the program
% before time out
MAX_ITER = 2000;
% The program stops if norm(omega)+norm(v) < eps
eps = 5*1e-5;
eps_2 = 1e-5;
%% Important background functions
% The 'hat' function, R^3\to\Skew_3
hat = @(omega)([0,-omega(3),omega(2);...
                omega(3),0,-omega(1);...
                -omega(2),omega(1),0]);

% Defining the group action
h_inv = @(R,T)([R',-R'*T;0,0,0,1]');
% SE_3 distance function
dist = @(R,T)(norm(logm([R,T;0,0,0,1]),'fro'));

%% Pre for loop preperation
% Color space inner product
% C = color_inner_product(fixed.Color, moving.Color, 1e-5);
% C = feature_inner_product(fixed.Color, moving.Color, 1e-5);
C = se_kernel(double(fixed.Color), double(moving.Color), 96, 1);
param.C = C;
% Initializing ICs
if nargin > 2
    R = varargin{1}(1:3,1:3); T = varargin{1}(1:3,4);
else
    R = eye(3); T = zeros(3,1);
end
%% The for loop
for k = 1:MAX_ITER
    % Construct the omega and v
    % The point clouds are fixed (x) and moving (z)
    TRANSFORM = h_inv(R,T); % Our current transformation
    %disp(TRANSFORM);
    tform = affine3d(TRANSFORM);
    moved = pctransform(moving,tform);
    [omega,v,step] = transform_identity_NR_4_sparse(fixed,moved,param);
    dt = step;% disp(dt);
    % Stop if the step size is too small
    if max(norm(omega),norm(v)) < eps
        break;
    end
    
    % Integrating
    th = norm(omega); homega = hat(omega); %disp(homega); disp(dt);
    dR = eye(3) + (sin(dt*th)/th)*homega + ((1-cos(dt*th))/th^2)*homega^2;
    dT = (dt*eye(3) + (1-cos(dt*th))/(th^2)*homega + ((dt*th-sin(dt*th))/th^3)*homega^2)*v;
    R_new = R*dR;
    T_new = R*dT+T;
    
    % Update the state
    R = R_new;
    T = T_new;
    distRT = dist(dR,dT);
    % Our other break
    if distRT < eps_2
        break;
    end

    if k > 3
        ell = 0.10;
        param.ell = ell;
    end
    if k > 10
        ell = 0.06;
        param.ell = ell;
    end
    if k > 20
        ell = 0.03;
        param.ell = ell;
    end
    
%     disp([k,max(norm(omega),norm(v)),eps,distRT,eps_2]);
    
end

TRANSFORM = h_inv(R,T);
tform = affine3d(TRANSFORM);