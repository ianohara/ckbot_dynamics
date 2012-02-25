function Rx = rotX(theta)
% Returns a 3x3 rotation matrix that corresponds to a rotation
% about the "X" or first axis.
%
% ARGUMENTS
%  theta - [rad] - angle through which to rotate
%
% RETURNS
%  Rx = Rotation matrix corresponding to a rotation about first axis by
%  angle theta.

Rx = [1,0,0;...
    0,cos(theta), -sin(theta);...
    0,sin(theta), cos(theta)];

end