% Setup, run, and verify a number of simulation steps and full
% simulations on simple chain configurations that can be verified
% by hand.

%% Single module (one fixed module, one module attached to the fixed module)
%    -> Equiv to a pendulum
clear all
close all

chain_single = [
    new_link('HT1','rotate', rotY(pi/2));
    new_link('HT1');
    new_link('HT1');
    new_link('HT2');
    %new_link('HT_head_cap');
    ];

N = size(chain_single, 1);

t_sim = 10;    % Simulate for t_sim [s]
num_s = 1000;  % Use this many timesteps

torque_history = zeros(N,num_s);  % At each timestep, the torque of each motor needs to be specified
q0 = zeros(N,1);
q0(1,1) = 0;
q0(3,1) = pi/6;
q0(4,1) = -pi/6;

qd0 = zeros(N,1);

sim = new_sim('steps', num_s, 'sim_time', t_sim, 'chain', chain_single, ...
    'torques', torque_history, 'q0', q0, 'qd0', qd0);

tic;
sim = run_sim(sim, num_s);
toc
draw_sim(sim);