function vels = forward_kinematics(chain)
% For a chain of CKBot links, calculate the linear velocity of both the CM
% and link end, along with the angular velocity of the link.
%
% ARGUMENTS
%  chain - A vector of links in the order that they're connected
% 
% RETURNS
%  vels - 3x3xN vector
%    vels(:,1,N) = velocity of forward link end in link frame
%    vels(:,2,N) = velocity of cm of link in link frame
%    vels(:,3,N) = angular velocity of link in link frame
%


N = length(chain);
vels = zeros(3,3,N);
vels(:,3,1) = [0;0;chain(1).qd];
for i = 2:N
    % Rotation from the previous link to this link's coordinates
    %  IE: transpose of going from this link to the previous
    R_prev = (rotZ(chain(i).q)*chain(i).R_jts)';
    % Vector from backward joint to forward joint
    r_jt_jt = -chain(i).r_im1 + chain(i).r_ip1;
    
    vels(:,1,i-1);
    v_prev = R_prev*vels(:,1,i-1);
    
    % Angular velocity - Rotate about the previous link's joint axis at our
    % motor speed (which is attached at our base to the previous joint
    % axis)
    vels(:,3,i) = R_prev*(vels(:,3,i-1) + chain(i).qd*chain(i-1).forward_joint_axis);
   
    % Linear velocity of our forward joint (for the next link to use)
    vels(:,1,i) = v_prev + cross(vels(:,3,i), r_jt_jt);
    
    % Linear velocity of center of mass
    vels(:,2,i) = v_prev + cross(vels(:,3,i), -chain(i).r_im1);
    
end

end