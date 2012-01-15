function M = get_joint_spatial_inertia_mat(link, r_cm_joint)
% Takes a link and returns the 6x6 spatial inertia matrix at the joint
% specified by r_cm_joint
%
% ARGUMENTS
%  link - with .I_cm, .mass defined
%  r_cm_joint - Vector from the link's CM to the joint
%
% RETURNS 
%  M - 6x6 sptial inertia matrix about the link
m = link.mass;
l_cross = get_cross_mat(r_cm_joint);
I_cm = link.I_cm;
I_j = I_cm - m*l_cross*l_cross;

M = [I_j, m*l_cross;...
    -m*l_cross, m*eye(3)];

end