function M_chain = get_spatial_inertia_mat(chain)
% Calculates the 6x6 spatial inertia matrix of each link
% in a chain (about inbound joint)
%
% ARGUMENTS
%  chain - a valid chain array full of link structures
%  
% RETURNS
%  M_chain - [6 x 6 x N] - spatial inertia matrix about the inbound joint.
%      This is a block matrix (with 3x3 blocks) of form:
%       M = [Rot Inertia, coupling; -coupling, Linear Inertia]
%
% NOTES:
%   1. This is equivalent to the shifting theorem except for
%      the full state (6x6) spatial algebra notation
%
% TODO:
%   1. Make sure L_tilde should use negative r_im1
%

N = size(chain,1);
M_chain = zeros(6,6,N);

for i=1:N
link = chain(i);
L_tilde = get_cross_mat(-link.r_im1);
m = link.m;

Jo = link.I_cm - m*L_tilde*L_tilde;

M_chain(:,:,i) = [Jo, m*L_tilde;...
    -m*L_tilde, m*eye(3)];
end 

end