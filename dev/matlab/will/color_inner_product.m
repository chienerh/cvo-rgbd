function C = color_inner_product(Cx, Cz, scale)
% Inner product matrix in color space.
% CX and CZ are n-by-3 and m-by-3 color matrices of each point cloud, 
% respectively, where n and m are number of observations.
% The resulting C matrix, is n-by-m and contains the inner product of all
% combinations of points from X and Z.
% scale is the parameter to scale the value of the inner product.


%  Each column of C is n-by-1 vector of inner product of all Cx rows with a
%  row of Cy. For example for the i-th column of C, we need 
% <Cx, cz_i'> = Cx * cz_i'. We can compute C all at once using Cx * Cz'.
C = scale * double(Cx) * double(Cz)';