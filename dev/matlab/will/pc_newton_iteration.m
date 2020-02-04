function tform = pc_newton_iteration(moving,fixed)
tic;
%% Defining the parameters
sigma = 0.1;
ell   = 0.05;
a2    = 7;
b2    = 7;
param = [sigma,ell,a2,b2];
% The maximum number of iterations before time-out
MAX_ITER = 250;
% The program halts if norm(omega)+norm(v) < eps
eps   = 5*1e-6;
eps_2 = 1e-6;
%% Important background functions
% The 'hat' function, R^3\to\Skew_3
hat = @(omega)([0,-omega(3),omega(2);...
                omega(3),0,-omega(1);...
                -omega(2),omega(1),0]);

% Defining the group action
h_inv = @(R,T)([R',-R'*T;0,0,0,1]');
% SE_3 distance function
dist = @(R,T)(norm(logm([R,T;0,0,0,1]),'fro'));

% Our initial guess
r0 = 0.00001*ones(3,1); t0 = 0.00001*ones(3,1);

% The group action
%h = @(R,T)([R,T;0,0,0,1]);
EXP = @(r,t)([eye(3)+(sin(norm(r))/norm(r))*hat(r)+(1-cos(norm(r)))/norm(r)^2*hat(r)^2,...
    (eye(3)+(1-cos(norm(r)))/norm(r)^2*hat(r)+(norm(r)-sin(norm(r)))/norm(r)^3*hat(r)^2)*t;0,0,0,1]);


%% The for loop
for k = 1:MAX_ITER
    TRANSFORM = EXP(-r0,-t0)';
    tform = affine3d(TRANSFORM);
    moved = pctransform(moving,tform);
    Lq0 = Lambda(fixed,moved,param);
    
    %If we have almost stopped moving
    if norm(Lq0) < eps
        break;
    end
    
    JLq0 = JLambda(fixed,moved,param,r0,t0);
    %save('Jac.mat','JLq0','Lq0'); break;
    q_plus = [r0;t0] - JLq0\Lq0;
    r0 = q_plus(1:3); t0 = q_plus(4:6);
    %disp([r0,t0]); 
    disp([max(eig(1/2*(JLq0+JLq0'))),norm(JLq0\Lq0),norm(Lq0),k]);

%     if k > 3
%         ell = 0.10;
%         param = [sigma,ell,a2,b2];
%     end
%     if k > 10
%         ell = 0.06;
%         param = [sigma,ell,a2,b2];
%     end
%     if k > 20
%         ell = 0.03;
%         param = [sigma,ell,a2,b2];
%     end
end
toc;
end

% Calculates the gradient, i.e. R^6\to R^6
function Lq = Lambda(cloud_x,cloud_y,param)
sigma = param(1); ell = param(2); c = param(3); d = param(4);
length_x = size(cloud_x.Location,1);
length_y = size(cloud_y.Location,1);
K = @(x,y)(sigma^2*exp(-vecnorm(x-y,2,2).^2/(2*ell^2)));

c_ij = @(i,j)(1e-5*double(cloud_x.Color(i,:))*double(cloud_y.Color(j,:))');
A_ij = @(i,j)(c_ij(i,j).*K(cloud_x.Location(i,:),cloud_y.Location(j,:))');

AIJ = @(i)(A_ij(i,1:length_y)');
cloud_i = @(i)(repmat(cloud_x.Location(i,:),length_y,1));
omega_ij = @(i)(1/c*AIJ(i).*cross(cloud_i(i),cloud_y.Location(:,:),2));
v_ij = @(i)(1/d*AIJ(i).*(cloud_y.Location(:,:)-cloud_x.Location(i,:)));

% Sum them up
omega_i = zeros(length_x,3); v_i = omega_i;
parfor i = 1:length_x
    k = i;

    partial_omega = sum(omega_ij(k));
    partial_v = sum(v_ij(k));
    
    omega_i(i,:) = partial_omega;
    v_i(i,:) = partial_v;
end
omega = sum(omega_i);
v = sum(v_i);
Lq = [omega';v'];
end

% Calculating the Jacobian of Lambda
function JLq = JLambda(cloud_x,cloud_y,param,r0,t0)
% Preliminary stuff
sigma = param(1); ell = param(2); c = param(3); d = param(4);
length_x = size(cloud_x.Location,1);
length_y = size(cloud_y.Location,1);
hat = @(omega)([0,-omega(3),omega(2);...
                omega(3),0,-omega(1);...
                -omega(2),omega(1),0]);
omega = hat(r0);
e1 = [1;0;0]; e2 = [0;1;0]; e3 = [0;0;1];
theta = norm(r0);
% Next comes the \hat{omega} partials
partial_w_1  = [0,0,0;0,0,-1;0,1,0];
partial_w_2  = [0,0,1;0,0,0;-1,0,0];
partial_w_3  = [0,-1,0;1,0,0;0,0,0];
partial_w2_1 = [0,r0(2),r0(3);r0(2),-2*r0(1),0;r0(3),0,-2*r0(1)];
partial_w2_2 = [-2*r0(2),r0(1),0;r0(1),0,r0(3);0,r0(3),-2*r0(2)];
partial_w2_3 = [-2*r0(3),0,r0(1);0,-2*r0(3),r0(2);r0(1),r0(2),0];
% Now comes the partials of the \tilde{z}_j
diff_z_r1 = @(j)((sin(theta)-theta*cos(theta))/theta^2*r0(1)/theta*omega-...
    sin(theta)/theta*partial_w_1 + (theta*sin(theta)+2*(cos(theta)-1))/theta^3*r0(1)/theta*omega^2+...
    (1-cos(theta))/theta^2*partial_w2_1)*cloud_y.Location(j,:)' -...
    (-(theta*sin(theta)+2*(cos(theta)-1))/theta^3*r0(1)/theta*omega + ...
    (cos(theta)-1)/theta*partial_w_1 + (3*sin(theta)-theta*(cos(theta)+2))/theta^4*r0(1)/theta*omega^2+...
    (theta-sin(theta))/theta^3*partial_w2_1)*t0;
diff_z_r2 = @(j)((sin(theta)-theta*cos(theta))/theta^2*r0(2)/theta*omega-...
    sin(theta)/theta*partial_w_2 + (theta*sin(theta)+2*(cos(theta)-1))/theta^3*r0(2)/theta*omega^2+...
    (1-cos(theta))/theta^2*partial_w2_2)*cloud_y.Location(j,:)' -...
    (-(theta*sin(theta)+2*(cos(theta)-1))/theta^3*r0(2)/theta*omega + ...
    (cos(theta)-1)/theta*partial_w_2 + (3*sin(theta)-theta*(cos(theta)+2))/theta^4*r0(2)/theta*omega^2+...
    (theta-sin(theta))/theta^3*partial_w2_2)*t0;
diff_z_r3 = @(j)((sin(theta)-theta*cos(theta))/theta^2*r0(3)/theta*omega-...
    sin(theta)/theta*partial_w_3 + (theta*sin(theta)+2*(cos(theta)-1))/theta^3*r0(3)/theta*omega^2+...
    (1-cos(theta))/theta^2*partial_w2_3)*cloud_y.Location(j,:)' -...
    (-(theta*sin(theta)+2*(cos(theta)-1))/theta^3*r0(3)/theta*omega + ...
    (cos(theta)-1)/theta*partial_w_3 + (3*sin(theta)-theta*(cos(theta)+2))/theta^4*r0(3)/theta*omega^2+...
    (theta-sin(theta))/theta^3*partial_w2_3)*t0;
diff_z_t1 = @(j)-(eye(3)-(1-cos(theta))/theta^2*omega+(theta-sin(theta))/theta^3*omega^2)*e1;
diff_z_t2 = @(j)-(eye(3)-(1-cos(theta))/theta^2*omega+(theta-sin(theta))/theta^3*omega^2)*e2;
diff_z_t3 = @(j)-(eye(3)-(1-cos(theta))/theta^2*omega+(theta-sin(theta))/theta^3*omega^2)*e3;
% Now comes the actual Jacobian
K = @(x,y)(sigma^2*exp(-vecnorm(x-y,2,2).^2/(2*ell^2)));
c_ij = @(i,j)(1e-5*double(cloud_x.Color(i,:))*double(cloud_y.Color(j,:))');
A_ij = @(i,j)(c_ij(i,j).*K(cloud_x.Location(i,:),cloud_y.Location(j,:))');
% omega
diff_omega_1 = @(i,j)(1/(c*ell^2)*A_ij(i,j)*(1/ell^2*dot(cloud_x.Location(i,:)-cloud_y.Location(j,:),diff_z_r1(j))*...
    cross(cloud_x.Location(i,:),cloud_y.Location(j,:))'+cross(cloud_x.Location(i,:)',diff_z_r1(j))));
diff_omega_2 = @(i,j)(1/(c*ell^2)*A_ij(i,j)*(1/ell^2*dot(cloud_x.Location(i,:)-cloud_y.Location(j,:),diff_z_r2(j))*...
    cross(cloud_x.Location(i,:),cloud_y.Location(j,:))'+cross(cloud_x.Location(i,:)',diff_z_r2(j))));
diff_omega_3 = @(i,j)(1/(c*ell^2)*A_ij(i,j)*(1/ell^2*dot(cloud_x.Location(i,:)-cloud_y.Location(j,:),diff_z_r3(j))*...
    cross(cloud_x.Location(i,:),cloud_y.Location(j,:))'+cross(cloud_x.Location(i,:)',diff_z_r3(j))));
diff_omega_4 = @(i,j)(1/(c*ell^2)*A_ij(i,j)*(1/ell^2*dot(cloud_x.Location(i,:)-cloud_y.Location(j,:),diff_z_t1(j))*...
    cross(cloud_x.Location(i,:),cloud_y.Location(j,:))'+cross(cloud_x.Location(i,:)',diff_z_t1(j))));
diff_omega_5 = @(i,j)(1/(c*ell^2)*A_ij(i,j)*(1/ell^2*dot(cloud_x.Location(i,:)-cloud_y.Location(j,:),diff_z_t2(j))*...
    cross(cloud_x.Location(i,:),cloud_y.Location(j,:))'+cross(cloud_x.Location(i,:)',diff_z_t2(j))));
diff_omega_6 = @(i,j)(1/(c*ell^2)*A_ij(i,j)*(1/ell^2*dot(cloud_x.Location(i,:)-cloud_y.Location(j,:),diff_z_t3(j))*...
    cross(cloud_x.Location(i,:),cloud_y.Location(j,:))'+cross(cloud_x.Location(i,:)',diff_z_t3(j))));
% v
diff_v_1 = @(i,j)(1/(d*ell^2)*A_ij(i,j)*(1/ell^2*dot(cloud_x.Location(i,:)-cloud_y.Location(j,:),diff_z_r1(j))*...
    (cloud_x.Location(i,:)-cloud_y.Location(j,:))'-diff_z_r1(j)));
diff_v_2 = @(i,j)(1/(d*ell^2)*A_ij(i,j)*(1/ell^2*dot(cloud_x.Location(i,:)-cloud_y.Location(j,:),diff_z_r2(j))*...
    (cloud_x.Location(i,:)-cloud_y.Location(j,:))'-diff_z_r2(j)));
diff_v_3 = @(i,j)(1/(d*ell^2)*A_ij(i,j)*(1/ell^2*dot(cloud_x.Location(i,:)-cloud_y.Location(j,:),diff_z_r3(j))*...
    (cloud_x.Location(i,:)-cloud_y.Location(j,:))'-diff_z_r3(j)));
diff_v_4 = @(i,j)(1/(d*ell^2)*A_ij(i,j)*(1/ell^2*dot(cloud_x.Location(i,:)-cloud_y.Location(j,:),diff_z_t1(j))*...
    (cloud_x.Location(i,:)-cloud_y.Location(j,:))'-diff_z_t1(j)));
diff_v_5 = @(i,j)(1/(d*ell^2)*A_ij(i,j)*(1/ell^2*dot(cloud_x.Location(i,:)-cloud_y.Location(j,:),diff_z_t2(j))*...
    (cloud_x.Location(i,:)-cloud_y.Location(j,:))'-diff_z_t2(j)));
diff_v_6 = @(i,j)(1/(d*ell^2)*A_ij(i,j)*(1/ell^2*dot(cloud_x.Location(i,:)-cloud_y.Location(j,:),diff_z_t3(j))*...
    (cloud_x.Location(i,:)-cloud_y.Location(j,:))'-diff_z_t3(j)));
% J
J = @(i,j)([diff_omega_1(i,j)',diff_v_1(i,j)';...
            diff_omega_2(i,j)',diff_v_2(i,j)';...
            diff_omega_3(i,j)',diff_v_3(i,j)';...
            diff_omega_4(i,j)',diff_v_4(i,j)';...
            diff_omega_5(i,j)',diff_v_5(i,j)';...
            diff_omega_6(i,j)',diff_v_6(i,j)']);
% Sum them up
JLq = zeros(6,6);
parfor i = 1:length_x
    k = i;
    JLq_i = zeros(6,6,length_y);
    for j = 1:length_y
        JLq_i(:,:,j) = J(k,j);
    end
    JLq = JLq + sum(JLq_i,3);
end
    
end