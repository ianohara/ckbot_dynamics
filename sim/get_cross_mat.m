function C = get_cross_mat(r)
% Returns the 3x3 cross product matrix corresponding to the vector r
%  IE: Cv = cross(r,v)
%
% ARGUMENTS
%  r - 3x1 or 1x3 - Vector to convert to cross matrix form

C = [0,    -r(3),  r(2); ...
     r(3),  0,    -r(1); ...
    -r(2),  r(1),  0];
end