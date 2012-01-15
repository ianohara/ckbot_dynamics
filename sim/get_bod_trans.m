function phi = get_bod_trans(r)
% Returns the 6x6 composite body transform
% corresponding to the vector r.  This is a spatial algebra operator
% and is specified in "Unified Formulation..." by Jain, Abhinandan
%
% ARGUMENTS:
%
% RETURNS:
%  phi - 6x6 spatial composite body transform matrix

I = eye(3);
Z = zeros(3);
C = get_cross_mat(r);
phi = [I, C; Z, I];

end