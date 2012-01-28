function [G_all, mu_all, a_all] = tip_base_step(chain, s, T)
%
% Calculates the values needed to do base to tip traversal for this
%   state (which calculates the acceleration of state) for 
%
% ARGUMENTS
%   sim - the simulation structure.  See sim_doc.txt for documentation.
%           Need: a chain configuration, and
%           sim.T(N,num_t) defined (torque command time history to use in
%           simulation)
%
% RETURNS
%
%  
% NOTES: 
%   1. Everything should be done in the inertial frame
%   2. In spatial algebra notation for dynamics, we use 6x1 vectors
%      (much like twists), except the 3 angular components are first
%      and the 3 linear components are second.
%
% TODO:
% 

grav = [0;0;0;0;0;-9.81];
pp = zeros(6); % Seed for the p+ (ie: p+ at link N+1)
zp = zeros(6,1);  % Seed for z+ (ie: z+ at link N+1)

% Stuff needed from sim structure
N = size(chain,1);
q = s(1:N);
qd = s(N+1:end);

% Initialize Return Variables
G_all = zeros(6*N,1);
mu_all = zeros(N,1);
a_all = zeros(6*N,1);

% Temporary hack?  My rotation derivation functions use the qs stored
% in the chain.
chain = propogate_angles_and_rates(chain,q,qd);

% link N = tip, link 1 = base
for i = N:-1:1
    
    cur = chain(i);
   
    R_cur = get_chain_pos_rot(chain, i); % Rotation from current link to inertial
    r_i_ip = R_cur*(cur.r_ip1 - cur.r_im1); % Vector from current joint to outbound joint
    phi = get_bod_trans(r_i_ip);  % Spatial transform operator from this joint to outbound joint
    
    r_i_cm = R_cur*(cur.r_im1);
    phi_cm = get_bod_trans(r_i_cm);
        
    M = get_spatial_inertia_mat(cur);  % 6x6 inertia matrix about inbound joint
    
    p_ind = get_block_indicies(i);
    
    p_cur = phi*pp*phi' + M;
    
    H_b_frame_star = get_joint_mat(cur)';
    H_w_frame_star = [R_cur, zeros(3);zeros(3),eye(3)]*H_b_frame_star;
    H = H_w_frame_star';
    
    D = H*p_cur*H';
    G = p_cur*H'*(1/D);
    
    tau_tilde = eye(6) - G*H;
    pp = tau_tilde*p_cur; % Set p+ for next time around loop
    
    omega = get_angular_vel(chain, i, qd(i));
    omega_cross = get_cross_mat(omega);
    
    b = [omega_cross*chain(i).I_cm*omega;...
        chain(i).m*omega_cross*omega_cross*(-cur.r_im1)];
    a = [0; 0; 0; omega_cross*omega_cross*(-cur.r_im1)];
    
    z = phi*zp + p_cur*a+b + phi_cm*chain(i).m*grav;
    epsilon = T(i) - H*z;
    
    mu = (1/D)*epsilon;
    zp = z + G*epsilon; % Setting z+ for the next time around loop

    % Returned for the base to tip traversal
    G_all(p_ind) = G;
    mu_all(i) = mu;
    a_all(p_ind) = a;
    
%     if (exist('DEBUG_MSG') || exist('DEBUG_TIP_BASE'))
%         fprintf('--------- Link %d -----------\n', i)
%         fprintf('Chain Description:\n  (%d=tip, 1=base)\n', N);
%         fprintf('The correction force at this link''s joint is:\n');
%         print_vec(z);
%         fprintf('The correction force through to the next link is:\n');
%         print_vec(zp);
%         fprintf('The Joint matrix is:\n');
%         print_vec(H')
%         fprintf('The correction force going through the inbound joint is:\n');
%         epsilon
%         fprintf('G is:\n');
%         G
%         fprintf('D is:\n');
%         D
%     end
end

end