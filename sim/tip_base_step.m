function [G_all, mu_all, a_all] = tip_base_step(chain, s, T)
%
% Calculates the values needed to do base to tip traversal for this
%   state (which calculates the acceleration of state) for 
%
% ARGUMENTS
%   chain - array of links (chain(1) = base, chain(N) = tip)
%   s - Current state [2N x 1]
%   T - Current motor torques [N x 1]
%
% RETURNS:
%  Intermediate results needed by the base to tip recursion step.
%    G_all - [6N x 1]
%    mu_all - [N x 1]
%    a_all - [6N x 1]
%  
% NOTES: 
%   1. Everything should be done in the inertial frame
%   2. In spatial algebra notation for dynamics, we use 6x1 vectors
%      (much like twists), except the 3 angular components are first
%      and the 3 linear components are second.
%
% TODO/BUGS:
%   1. get_rotation... and get_omega... are eating up ~25% of computation time
%   in this function.  They should be moved outside of the loop and changed
%   so that they calculate the values for the entire arm at once, then
%   index to the correct portions within the loop.
% 
%   2. The tip module acts as if gravity is acting in the opposite
%   direction.  Also, The other modules react properly (ie: stable
%   equilibrium is down) when g = 9.81 (instead of the expected -9.81).)
%   No idea why the tip module has gravity act in the opposite sense than
%   the rest of the chain.


grav = [0;0;0;0;0;9.81];
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
    
    % Vector from inbound joint to CM of this link (in inertial coords)
    r_i_cm = R_cur*(-cur.r_im1);
    % Spatial transformation from inbound joint to CM
    phi_cm = get_bod_trans(r_i_cm);
        
    M = get_spatial_inertia_mat(cur);  % 6x6 inertia matrix about inbound joint
    
    p_ind = get_block_indicies(i);
    
    % Articulated body spatial inertia of this link
    p_cur = phi*pp*phi' + M;
    
    % Joint matrix.  H is in the inertial coordinate sys (like everything
    % used in this algorithm should be)
    H_b_frame_star = get_joint_mat(cur)';
    H_w_frame_star = [R_cur, zeros(3);zeros(3),eye(3)]*H_b_frame_star;
    H = H_w_frame_star';
    
    % Joint "Inertia"
    D = H*p_cur*H';
    
    G = p_cur*H'*(1/D);
    
    tau_tilde = eye(6) - G*H;
    
    % Set p+ for next time around loop
    pp = tau_tilde*p_cur; 
    
    omega = get_angular_vel(chain, i, qd(i));
    omega_cross = get_cross_mat(omega);
    
    % The Coriolis and Gyroscopic terms
    b = [omega_cross*chain(i).I_cm*omega;...
        chain(i).m*omega_cross*omega_cross*(-cur.r_im1)];
    a = [0; 0; 0; omega_cross*omega_cross*(-cur.r_im1)];
    
    % Spatial compensation force
    z = phi*zp + p_cur*a+b + phi_cm*chain(i).m*grav;
    
    % Velocity dependent joint force (ie: damping)
    % NOTE: Not in JPL paper.  Added by IMO
    c = -cur.damping*qd(i);
    
    % Correction force through the inbound joint (including the torque
    % added due to motor torque and damping)
    epsilon = T(i) + c - H*z;
    
    % Relative joint acceleration
    mu = (1/D)*epsilon;
    zp = z + G*epsilon; % Setting z+ for the next time around loop

    % Returned for the base to tip traversal
    G_all(p_ind) = G;
    mu_all(i) = mu;
    a_all(p_ind) = a;
    
end

end