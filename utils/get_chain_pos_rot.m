function R = get_chain_pos_rot(chain, pos)
% Form the rotation matrix that brings the link at
%  position, pos, in the chain to the inertial frame
%
% ARGUMENTS
%  chain - An array of links where chain(1) is the base link
%          and chain(end) is the tip link
%  pos - An integer in the range 1 <= pos <= size(chain,1)
%        that corresponds the link we want the rotation mat for
% RETURNS
%  R - [3 x 3] - Rotation from chain(pos) to the inertial frame
%
% TODO
%  1. Get rid of for loop.
%  2. Memoize

if ((pos < 1) || (pos > size(chain,1)))
    throw('pos out of range.');
end

R = eye(3);

for i=1:pos
   R = R*rotZ(chain(i).q)*chain(i).R_jts;
end

end