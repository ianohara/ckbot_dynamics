function success = draw_lin_vels(chain)
% Draw the linear velocities of a chain's CMs based on the current state
%
% ARGUMENTS
%  chain - vector of links in the order they're attached
%
% RETURNS
%  success - 1 for success, 0 for failure

vels = forward_kinematics(chain);

R = eye(3);
r_base = [0; 0; 0];

for i=1:length(chain)
    R = R*rotZ(chain(i).q)*chain(i).R_jts;
    r_cm = r_base - R*(chain(i).r_im1);
    r_base = r_cm + R*(chain(i).r_ip1);
    
    draw_arrow(r_cm, r_cm+R*vels(:,1,i));
end

end