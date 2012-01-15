function H = get_joint_mat(link)
% Returns the 1X6 joint matrix that maps the joint velocity
%  of this link to the relative spatial velocity between this
%  link and the last
%
% ARGUMENTS
%  link - valid link structure
% 
% RETURNS
%  H - 1x6 - Matrix such that delta_v = Transpose(H)*link.qd
%
% TODO:
%  1. KEY: Check to make sure that this doesn't need to be in
%     the inertial frame
%
% NOTES: (DEPENDS ON TODO 1.) - In the relative frame, the joint
%         Always rotates wrt the previous link's z axis in its 
%         reference frame

Hprev = [0;0;1]; % Only rotational freedom in this sytem
Ht = link.R_jts'*Hprev;
H = [0,0,0, Ht']; % Should be 1x6

end