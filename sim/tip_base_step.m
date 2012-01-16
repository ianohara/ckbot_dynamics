function sim = tip_base_step(sim)
%
% Calculates P and z using the current system step (sim.s) state.
%
% ARGUMENTS
%   sim - the simulation structure.  See sim_doc.txt for documentation.
%           Need: a chain configuration, and
%           sim.T(N,num_t) defined (torque command time history to use in
%           simulation)
%
% RETURNS
%   sim - the simulation structure with sim.p,sim.z,sim.s_vars.*, filled
%   out for the current step of the system (sim.s)
%  
% NOTES: 
%   1. Everything should be done in the inertial frame
%
% TODO:
% 
grav = [0;0;-9.81;0;0;0];
pp = zeros(6); % Seed for the p+ (ie: p+ at link N+1)
zp = zeros(6,1);  % Seed for z+ (ie: z+ at link N+1)
N = size(sim.chain,1);
s = sim.s;  % Current simulation step

% link N = tip, link 1 = base
for i = N:-1:1
    cur = sim.chain(i);
   
    R_cur = get_chain_pos_rot(sim.chain, i); % Rotation from current link to inertial
    r_i_ip = R_cur*(cur.r_ip1 - cur.r_im1); % Vector from current joint to outbound joint
    phi = get_bod_trans(r_i_ip);  % Spatial transform operator from this joint to outbound joint
    
    M = get_spatial_inertia_mat(cur);
    
    p_ind = get_block_indicies(size(sim.p), i, s);
    p_cur = phi*pp*phi' + M;
    sim.p(p_ind,p_ind, s) = p_cur;
    
    H = get_joint_mat(cur);
    
    D = H*p_cur*H';
    G = p_cur*H'*(1/D);
    
    tau_tilde = eye(6) - G*H;
    pp = tau_tilde*p_cur; % Setting p+ for next time around loop
    
    omega = get_angular_vel(sim.chain, i, sim.qd(i,s));
    omega_cross = get_cross_mat(omega);
    
    b = [omega_cross*sim.chain(i).I_cm*omega;...
        sim.chain(i).m*omega_cross*omega_cross*(-cur.r_im1)];
    a = [0; 0; 0; omega_cross*omega_cross*(-cur.r_im1)];
    
    z = phi*zp + p_cur*a+b - sim.chain(i).m*grav; % TODO: Right place for grav here?
    epsilon = sim.T(i,s) - H*z;
    
    mu = (1/D)*epsilon;
    zp = z + G*epsilon; % Setting z+ for the next time around loop

    % Store the stuff we need for the base to tip traversals
    sim.s_vars.G(p_ind) = G;
    sim.s_vars.mu(i) = mu;
end

end