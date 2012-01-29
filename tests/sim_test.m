% Setup, run, and verify a number of simulation steps and full
% simulations on simple chain configurations that can be verified
% by hand.

%% Single module (one fixed module, one module attached to the fixed module)
%    -> Equiv to a pendulum
clear all
close all
chain_single = [
    new_link('HT1','rotate', rotY(pi/2));
    new_link('HT2');
    new_link('HT1');
    new_link('HT_head_cap');
    ];

N = size(chain_single, 1);

t_sim = 30;    % Simulate for t_sim [s]
num_s = 3000;  % Use this many timesteps

torque_history = zeros(N,num_s);  % At each timestep, the torque of each motor needs to be specified
q0 = zeros(N,1);
q0(2,1) = pi/6;

qd0 = zeros(N,1);

sim = new_sim('steps', num_s, 'sim_time', t_sim, 'chain', chain_single, ...
    'torques', torque_history, 'q0', q0, 'qd0', qd0);

tic;
% Calculate the rate of change of state at these initial conditions
for i=1:num_s-1
    [q_n,qd_n] = step_sim(sim.chain, sim.q(:,i), sim.qd(:,i), sim.T(:,i), sim.dt);
    sim.q(:,i+1) = q_n;
    sim.qd(:,i+1) = qd_n;
    sim.s = sim.s+1;
end
toc
%% 