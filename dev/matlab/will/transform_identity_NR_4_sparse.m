% A script that computes the Lie algebra transformation elements
% The parameters needed are:
% 1) \sigma
% 2) \ell
% 3) c
% 4) d
% An input vector called param = [sigma,ell,c,d] is needed
% The outputs, [omega,v], are a 3 by 2 matrix

function [omega,v,step] = transform_identity_NR_4_sparse(cloud_x,cloud_y,param)
%cloud_x = fixed; cloud_y = moving;
% Extract the parameters
sigma = param.sigma; ell = param.ell; c = param.a2; d = param.b2;

% Extract point cloud information:
length_x = size(cloud_x.Location,1);
length_y = size(cloud_y.Location,1);

cloud_x = double(cloud_x.Location);
cloud_y = double(cloud_y.Location);

% Construct our kernel and coefficient
K = se_kernel(cloud_x, cloud_y, ell, sigma^2);
% K(K < 1e-3) = 0;
A = param.C .* K;
A(A < 1e-3) = 0;
A = sparse(double(A));

% The 'hat' function, R^3\to\Skew_3
hat = @(omega)([0,-omega(3),omega(2);...
                omega(3),0,-omega(1);...
                -omega(2),omega(1),0]);

% Sum them up
omega = zeros(1,3); v = omega;
ids = cell(length_x,1);
for i = 1:length_x
        ids{i} = find(A(i,:));
        Ai = nonzeros(A(i,:))';
        cloud_yi = cloud_y(ids{i},:);
        partial_omega = 1/c * Ai * ...
            cross(repmat(cloud_x(i,:), length(ids{i}),1), cloud_yi ,2);
        partial_v = 1/d * Ai * ...
            (cloud_yi - cloud_x(i,:));

        omega = omega + partial_omega;
        v = v + partial_v;
end
% omega = omega / ell^2;
% v = v / ell^2;
% The derivatives information
omega_hat = hat(omega);
OMEGA = repmat(omega,length_y,1);
xi_z = (cross(OMEGA,cloud_y(:,:),2) + v);
xixi_z = (omega_hat^2 * (cloud_y(:,:)') + omega_hat * v')';
xixixi_z = (omega_hat^3 * (cloud_y(:,:)') + omega_hat^2 * v')';
xixixixi_z = (omega_hat^4 * (cloud_y(:,:)') + omega_hat^3 * v')';

% The coefficient information
normxiz2 = vecnorm(xi_z,2,2).^2;
dotxizxixi = 2 * sum(-xi_z .* xixi_z,2);
epsil_const = vecnorm(xixi_z,2,2).^2 + 2 * sum(-xi_z .* -xixixi_z,2);

% Calculate the derivatives
b_i = zeros(length_x,1); c_i = b_i; d_i = b_i; e_i = b_i;
for i = 1:length_x
        y_i = cloud_x(i,:) - cloud_y(ids{i},:);
        beta_i = -sum(xi_z(ids{i},:) .* y_i,2)/ell^2;
        gamma_i = -1/(2*ell^2)*(normxiz2(ids{i}) + 2 * sum(xixi_z(ids{i},:) .* y_i,2));
        delta_i = 1/(2*ell^2)*(dotxizxixi(ids{i},:) + 2 * sum(-xixixi_z(ids{i},:) .* y_i,2));
        epsil_i = -1/(2*ell^2)*(epsil_const(ids{i},:) + 2 * sum(xixixixi_z(ids{i},:) .* y_i,2));
        
        Ai = nonzeros(A(i,:))';
        b_i(i) = Ai * beta_i;
        beta2_i = beta_i .* beta_i;
        c_i(i) = Ai * (gamma_i + beta2_i/2);
        beta_gamma_i = beta_i .* gamma_i;
        beta3_i = beta2_i .* beta_i;
        d_i(i) = Ai * (delta_i + beta_gamma_i + beta3_i/6);
        e_i(i) = Ai * (epsil_i + beta_i .* delta_i + ...
            1/2*beta_i .* beta_gamma_i + 1/2*gamma_i .* gamma_i + 1/24*beta_i .* beta3_i);
end

B = sum(b_i)';
C = sum(c_i)';
D = sum(d_i)';
E = sum(e_i)';
% Displaying the found coeff

coef2 = [4*E,3*D,2*C,B];
% The step
r = roots(coef2);

step = min( min(r((r>0)&(r==real(r)))) , 0.8);
if isempty(step)
    step = 2*1e-1;
end
omega = omega';
v = v';