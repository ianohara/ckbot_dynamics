function R_chain = get_chain_pos_rot(chain)
% Form the rotation matricies that brings each link
% in the chain from its own coordinate frame to the inertial
% coordinate frame.
%
% ARGUMENTS
%  chain - An array of links where chain(1) is the base link
%          and chain(end) is the tip link
% RETURNS
%  R - [N x 3 x 3] - Rotation from chain(pos) to the inertial frame
%

N = size(chain,1);
R = chain(1).init_rotation;
R_chain = zeros(3,3,N);

for i=1:N
   R = R*rotZ(chain(i).q)*chain(i).R_jts;
   R_chain(:,:,i) = R;
end
end