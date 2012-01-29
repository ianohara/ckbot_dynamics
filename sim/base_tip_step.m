function qdd = base_tip_step(chain, s, T, G, mu, a)
% Calculates the joint accelerations based on the current
% system state, input torques, G, mu, and 'a' calculated in
% the tip_base_step for this step.
%
% ARGUMENTS:
%  sim - the simulation structure.  See sim_doc.txt for documentation
%
% RETURNS:
%  
% qdd - nx1 array of joint accelerations

N = size(chain,1);

alpha = zeros(6,1);  % The seed acceleration (ie: the base joint does not rotate
                     % wrt the ground, so we use this to seed the base to tip
                     % recursion)
q = s(1:N);
qd = s(N+1:end);
qdd = NaN(N,1);

chain = propogate_angles_and_rates(chain, q, qd);

for i = 1:N
    cur = chain(i);
    
    R_cur = get_chain_pos_rot(chain, i);
    r_i_ip = R_cur*(cur.r_ip1 - cur.r_im1);
    phi = get_bod_trans(r_i_ip);   % From outbound to inbound
  
    alpha_p = phi'*alpha;  % inbound alpha transformed to outbound frame
    
    p_ind = get_block_indicies(i);
    
    qdd(i) = mu(i) - G(p_ind)'*alpha_p;
    
    H = get_joint_mat(cur);
    alpha = alpha_p + H'*qdd(i) + a(p_ind);     
% 
%     if (exist('DEBUG_MSG') || exist('DEBUG_BASE_TIP'))
%        fprintf('----------- LINK %d ------------\n', i);
%        fprintf('Chain Description:\n (1=base, %d=tip)\n', N);
%        fprintf('Spatial acceleration at this link''s joint:\n');
%        print_vec(alpha);
%         
%     end
end

end