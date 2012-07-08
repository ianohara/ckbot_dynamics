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
% 

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

R_chain = get_chain_pos_rot(chain); % Nx3x3 array of rotation matricies for each link in the chain

% DEBUG
global counter;
counter = counter + 1;

% link N = tip, link 1 = base
for i = N:-1:1
    cur = chain(i);
    
    fprintf('Link %d (q=%2.2f, qd=%2.2f):\n', i, cur.q, cur.qd); % DEBUG

    R_cur = R_chain(:,:,i); % Rotation from current link to inertial
    r_i_ip = R_cur*(cur.r_ip1 - cur.r_im1); % Vector from current joint to outbound joint
    phi = get_bod_trans(r_i_ip);  % Spatial transform operator from this joint to outbound joint
    
    % Vector from inbound joint to CM of this link (in inertial coords)
    r_i_cm = R_cur*(-cur.r_im1);
    
    % Spatial transformation from CM to inbound joint
    phi_cm_op = get_bod_trans(r_i_cm);
     
    % Form the 6x6 inertia Matrix about the inbound joint
    % And the 3x3 inertia matrix about the inbound joint as well
    L_oc_tilde = get_cross_mat(r_i_cm);
    J_o = cur.I_cm - cur.m*L_oc_tilde*L_oc_tilde;
    
    M = [J_o, cur.m*L_oc_tilde; ...
         -cur.m*L_oc_tilde, cur.m*eye(3)]; % 6x6 inertia matrix about inbound joint
    
    M_cm = [cur.I_cm, zeros(3);
            zeros(3), cur.m*eye(3)];
     
    p_ind = get_block_indicies(i);
    
    % Articulated body spatial inertia of this link
    % phi transpose is on the RHS because we're bringing
    % pp from the outbound joint to the inbound joint,
    % and phi does the opposite.
    p_cur = phi*pp*phi' + M;
    
    % Joint matrix.  H is in the inertial coordinate sys (like everything
    % used in this algorithm should be)
    H_b_frame_star = get_joint_mat(cur)';
    H_w_frame_star = [R_cur, zeros(3);
                      zeros(3), R_cur]*H_b_frame_star;
    H = H_w_frame_star'

    
    % Joint "Inertia"
    D = H*p_cur*H';
    
    G = p_cur*H'*(1/D);
    
    tau_tilde = eye(6) - G*H;
    
    % Set p+ for next time around loop
    pp = tau_tilde*p_cur; 
    
    omega = get_angular_vel(chain, i);
    omega_cross = get_cross_mat(omega);
    
    % The Coriolis and Gyroscopic terms
    b = [omega_cross*J_o*omega;...
        chain(i).m*omega_cross*omega_cross*r_i_cm];  % pg 5
    
    % DEBUG from pg 9 and 76 of Jain book
    V_omega = [omega; 0; 0; 0];
    V_omega_tilde = [omega_cross, zeros(3);
                     zeros(3), omega_cross];
    disp('b from 76:');
    V_omega_tilde*M*V_omega
    %disp('a from 79:');
    %omega_delta = R_cur*(cur.qd*cur.forward_joint_axis); % Relative omega from this joint to next
    %a_book = [omega_cross*omega_delta;
              
    a = [0; ...
         0; ...
         0; ...
         omega_cross*omega_cross*r_i_cm];   % pg 4
    
    % Spatial compensation force at inbound joint.
    
    z = phi*zp + b + p_cur*a + phi_cm_op*M_cm*grav % DEBUG + M*phi_cm*grav;% + b;% + M*phi_cm*grav;
    b
    disp('a:');
    p_cur*a
    % Velocity dependent joint force (ie: damping)
    % NOTE: Not in JPL paper.  Added by IMO
    c = -cur.damping*qd(i);
    
    % Correction force through the inward joint (including the torque
    % added due to motor torque and damping)
    epsilon = T(i) + c - H*z;
    
    % Relative joint acceleration
    Mu = (1/D)*epsilon;
    zp = z + G*epsilon; % Setting z+ for the next time around loop

    % Returned for the base to tip traversal
    G_all(p_ind) = G;
    mu_all(i) = Mu;
    a_all(p_ind) = a;  
end

end