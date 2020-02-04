function rkhs_toy_example()
clc; clear all; close all

kernel = @(x,z) exp(-norm(x-z)^2);

alpha = [3 4 2.5 -2];
beta = [3 4 1.5 -2];

d = 1.5;
xt = linspace(0,3, 20)';
xs = linspace(0 + d,5 + d, 15)';

% pick random basis!
basis_idt = floor(size(xt,1) * rand(length(alpha),1)) + 1;

yt = [];
for j = 1:size(xt,1)
    for i = 1:length(alpha)
        yt(j,1) = alpha(i) * kernel(xt(j,:)', xt(basis_idt(i),:)');
    end
end

% pick random basis!
basis_ids = floor(size(xs,1) * rand(length(beta),1)) + 1;

ys = [];
for j = 1:size(xs,1)
    for i = 1:length(beta)
        ys(j,1) = beta(i) * kernel(xs(j,:)', xs(basis_ids(i),:)');
    end
end

figure; hold on
plot(xt, yt, xs, ys, '--', 'linewidth', 2)
plot(xt(basis_idt), yt(basis_idt), '+', xs(basis_ids), ys(basis_ids), 'o', 'markersize', 8)


p_target = xt; % target (fixed)
p_source = xs; % source (moving)

x0 = 0;

[x, fval, exitflag] = fminsearch(@cost, x0)

xs = xs - x;

figure; hold on
plot(xt, yt, xs, ys, '--', 'linewidth', 2)
plot(xt(basis_idt), yt(basis_idt), '+', xs(basis_ids), ys(basis_ids), 'o', 'markersize', 8)


% Cost function
    function f = cost(x)
        z = p_source - x(1);
        for ii = 1:length(alpha)
            for jj = 1:length(beta)
                f = -alpha(ii) * beta(jj) * kernel(p_target(basis_idt(ii)), z(basis_ids(jj)));
            end
        end
    end

end
