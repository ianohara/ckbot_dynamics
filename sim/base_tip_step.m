function qdd = base_tip_step(chain, s, T, G, mu, a)
% Calculates the joint accelerations based on the current
% system state, input torques, G, mu, and 'a' calculated in
% the tip_base_step for this step.
%
% ARGUMENTS:
%   chain - array of links
%   s - Current state [2N x 1]
%   T - Current motor torques [N x 1]
%   G - G calculated by tip_base_step(...) [6N x 1]
%   mu - mu calculated by tip_base_step(...) [N x 1]
%   a - a calculated by tip_base_step(...) [6N x 1]
%
% RETURNS:  
%   qdd - nx1 array of joint accelerations
%
% TODO:
%   Move get_chain_pos_rot(...) outside of the loop and change it to return
%   a 3x3N block matrix where each 3x3 block is a rotation matrix for a
%   link.  Then R_Cur inside the loop can be obtained by indexing this
%   block matrix.
%
N = size(chain,1);

alpha = zeros(6,1);  % The seed acceleration (ie: the base joint does not rotate
                     % wrt the ground, so we use this to seed the base to tip
                     % recursion)
q = s(1:N);
qd = s(N+1:end);
qdd = NaN(N,1);  % We calculate this.

chain = propogate_angles_and_rates(chain, q, qd);

for i = 1:N
    cur = chain(i);
    
    % Rotation of current link's coordinate frame to the inertial frame
    R_cur = get_chain_pos_rot(chain, i);
    
    % Vector from this link's inbound joint to its outbound joint
    r_i_ip = R_cur*(cur.r_ip1 - cur.r_im1);
    % Spatial tranformation operator from outbound joint to inbound joint
    phi = get_bod_trans(r_i_ip);
  
    % inbound alpha transformed to outbound frame (note phi transpose)
    % for next time around loop
    alpha_p = phi'*alpha;  
    
    p_ind = get_block_indicies(i);
    
    qdd(i) = mu(i) - G(p_ind)'*alpha_p;
    
    % Joint matrix.  H is in the inertial coordinate sys (like everything
    % used in this algorithm should be)
    H_b_frame_star = get_joint_mat(cur)';
    H_w_frame_star = [R_cur, zeros(3);zeros(3),eye(3)]*H_b_frame_star;
    H = H_w_frame_star';
    
    % Spatial acceleration of the link's inbound joint
    alpha = alpha_p + H'*qdd(i) + a(p_ind);
end

end