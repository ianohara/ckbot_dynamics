function Rz = rotZ(phi)
% Returns a 3x3 rotation matrix that corresponds to a rotation
% about the "Z" or third axis.
%
% ARGUMENTS
%  theta - [rad] - angle through which to rotate
%
% RETURNS
%  Rz = Rotation matrix corresponding to a rotation about third axis by
%  angle phi.

Rz = [cos(phi), -sin(phi), 0;...
    sin(phi), cos(phi), 0;...
    0, 0, 1];

end