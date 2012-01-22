% Setup, run, and verify a number of simulation steps and full
% simulations on simple chain configurations that can be verified
% by hand.

%% Single module (one fixed module, one module attached to the fixed module)
%    -> Equiv to a pendulum

chain_single = [
    new_link('HT_tail_cap','rotate', rotX(pi/2));
    new_link('HT1');
    new_link('HT_head_cap');
    ];

N = size(chain_single, 1);

t_sim = 2;    % Simulate for t_sim [s]
num_s = 100;  % Use 100 timesteps

torque_history = zeros(N,num_s);
q0 = zeros(N,1); %pi/6*ones(N,1);
q0(1,1) = 0;

qd0 = zeros(N,1);

qd0(1,1) = 1;

sim = new_sim('steps', num_s, 'sim_time', t_sim, 'chain', chain_single, ...
    'torques', torque_history, 'q0', q0, 'qd0', qd0);

% Calculate the rate of change of state at these initial conditions
sim = tip_base_step(sim);
sim = base_tip_step(sim);

%% 