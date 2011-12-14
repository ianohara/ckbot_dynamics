function Ry = rotY(psi)
% Returns a 3x3 rotation matrix that corresponds to a rotation
% about the "Y" or second axis.
%
% ARGUMENTS
%  psi - [rad] - angle through which to rotate
%
% RETURNS
%  Ry = Rotation matrix corresponding to a rotation about second axis by
%  angle psi.

Ry = [cos(psi), 0, sin(psi);...
    0,1,0;...
    -sin(psi), 0, cos(psi)];

end