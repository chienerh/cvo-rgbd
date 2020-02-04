function rkhs_toy_example_2D()
clc; clear all; close all

kernel = @(x,z) exp(-norm(x-z)^2);

alpha = [-3 -4 2.5 -2 1 -5 3 1 3 -2 4 2 3];
beta = [-3 -4 1.5 -2 1 -5 3 1 3 -2 4 2 3];

d = 0.5;
xt = linspace(-3,3, 25)';
[X1t, X2t] = meshgrid(xt, xt);
Xt = [X1t(:), X2t(:)];

xs = linspace(-2 + d,3 + d, 10)';
[X1s, X2s] = meshgrid(xs, xs);
Xs = [X1s(:), X2s(:)];

% pick random basis!
basis_idt = floor(size(Xt,1) * rand(length(alpha),1)) + 1;

yt = [];
for j = 1:size(Xt,1)
    for i = 1:length(alpha)
        yt(j,1) = alpha(i) * kernel(Xt(j,:)', Xt(basis_idt(i),:)');
    end
end

% pick random basis!
basis_ids = floor(size(Xs,1) * rand(length(beta),1)) + 1;

ys = [];
for j = 1:size(Xs,1)
    for i = 1:length(beta)
        ys(j,1) = beta(i) * kernel(Xs(j,:)', Xs(basis_ids(i),:)');
    end
end

figure; hold on
plot3(Xt(basis_idt,1), Xt(basis_idt,2), yt(basis_idt), 's', Xs(basis_ids,1), Xs(basis_ids,2), ys(basis_ids), 'o', 'markersize', 8)
surf(X1t, X2t, reshape(yt, size(X1t)), 'FaceAlpha',0.5)
surf(X1s, X2s, reshape(ys, size(X1s)), 'FaceAlpha',0.5)
axis auto, grid on, view(90, 90)

p_target = Xt; % target (fixed)
p_source = Xs; % source (moving)

x0 = [0, 0, 0];

[x, fval, exitflag] = fminsearch(@cost, x0)

Rot = [cos(x(3)) -sin(x(3)); 
    sin(x(3)) cos(x(3))];
        
Xs = (Rot' * Xs')' - (Rot' * [x(1); x(2)])';
X1s = reshape(Xs(:,1), size(X1s));
X2s = reshape(Xs(:,2), size(X2s));

figure; hold on
plot3(Xt(basis_idt,1), Xt(basis_idt,2), yt(basis_idt), 's', Xs(basis_ids,1), Xs(basis_ids,2), ys(basis_ids), 'o', 'markersize', 8)
surf(X1t, X2t, reshape(yt, size(X1t)), 'FaceAlpha',0.5)
surf(X1s, X2s, reshape(ys, size(X1s)), 'FaceAlpha',0.5)
axis auto, grid on, view(90, 90)


% Cost function
    function f = cost(x)
        R = [cos(x(3)) -sin(x(3));
            sin(x(3)) cos(x(3))];
        z = (R' * p_source')' - (R' * [x(1); x(2)])';
        for ii = 1:length(alpha)
            for jj = 1:length(beta)
                f = -alpha(ii) * beta(jj) * kernel(p_target(basis_idt(ii),:), z(basis_ids(jj),:));
            end
        end
    end

end
