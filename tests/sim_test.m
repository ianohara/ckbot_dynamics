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
    %new_link('HT_head_cap');
    ];

N = size(chain_single, 1);

t_sim = 2;    % Simulate for t_sim [s]
num_s = 1000;  % Use 100 timesteps

torque_history = zeros(N,num_s);
q0 = zeros(N,1); %pi/6*ones(N,1);
q0(2,1) = pi/60;

qd0 = zeros(N,1);

qd0(1,1) = 1;

sim = new_sim('steps', num_s, 'sim_time', t_sim, 'chain', chain_single, ...
    'torques', torque_history, 'q0', q0, 'qd0', qd0);

% Calculate the rate of change of state at these initial conditions
for i=1:num_s
    sim = step_sim(sim);
end

%% 