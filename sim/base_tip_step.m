function sim = base_tip_step(sim)
% Calculates the joint accelerations based on the current
% system state step (sim.s) defined for this simulation and the results
% of the tip to base recursion on this same state.
%
% ARGUMENTS:
%  sim - the simulation structure.  See sim_doc.txt for documentation
%
% RETURNS:
%  
% 

N = size(sim.chain,1);
s = sim.s;

alpha = zeros(6,1);  % The seed acceleration (ie: the base joint does not rotate
            % wrt the ground, so we use this to seed the base to tip
            % recursion)
% g = -9.81;
% alpha(3) = g;

for i = 1:N
    cur = sim.chain(i);
    
    R_cur = get_chain_pos_rot(sim.chain, i);
    r_i_ip = R_cur*(cur.r_ip1 - cur.r_im1);
    phi = get_bod_trans(r_i_ip);  % From outbound to inbound
  
    alpha_p = phi'*alpha;  % inbound alpha transformed to outbound frame
    
    p_ind = get_block_indicies(size(sim.p),i,s);
    
    sim.qdd(i,s) = sim.s_vars.mu(i) - sim.s_vars.G(p_ind)'*alpha_p;
    
    H = get_joint_mat(cur);
    alpha = alpha_p + H'*sim.qdd(i,s) + sim.s_vars.a(p_ind);     
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