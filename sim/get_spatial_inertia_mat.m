function M = get_spatial_inertia_mat(link)
% Calculates the 6x6 spatial inertia matrix of a link about
% its inbound joint
%
% ARGUMENTS
%  link - a valid link structure
%  
% RETURNS
%  M - [6 x 6] - spatial inertia matrix about the inbound joint.
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

L_tilde = get_cross_mat(-link.r_im1);
m = link.m;

Jo = link.I_cm - m*L_tilde*L_tilde;

M = [Jo, m*L_tilde;...
    -m*L_tilde, m*eye(3)];
end