function success = sim_test(varargin)

% Setup, run, and verify a number of simulation steps and full
% simulations on simple chain configurations that can be verified
% by hand

if(strcmp(varargin{1}, 'sim_num'))
    sim_num = varargin{2};
else
    sim_num = 0;
end

%% Single module (one fixed module, one module attached to the fixed module)
%    -> Equiv to a pendulum
if ((sim_num == 1) || (~sim_num))
fprintf('Running simulation number 1:\n');
close all
global sim

chain_single = [
    new_link('HT1','rotate',rotY(pi/2));
    new_link('HT1');
    ];

chain_single(1).damping = 1.0;

N = size(chain_single, 1);
t_sim = 10;
num_s = 1500;
torque_history = zeros(N, num_s);
q0 = zeros(N,1);
q0(2) = pi/2;

qd0 = zeros(N,1);

sim = new_sim('steps', num_s, 'sim_time', t_sim, 'chain', chain_single, ...
    'torques', torque_history, 'q0', q0, 'qd0', qd0);

tic;
sim = run_sim(sim, num_s);
toc;

draw_sim(sim, varargin{:});
success = 1;

elseif ((sim_num == 2) || (~sim_num))
%% Multiple Modules
close all

global sim

chain_single = [
    new_link('HT1','rotate', rotY(pi/2));
    new_link('HT2');
    new_link('HT1');
    new_link('HT2');
    new_link('HT1');
    new_link('HT2');
    new_link('HT1');
    new_link('HT2');
    ];

N = size(chain_single, 1);

t_sim = 10;    % Simulate for t_sim [s]
num_s = 100;  % Use this many timesteps

torque_scale = 1;
torque_omega = 1/20;

torque_history = zeros(N,num_s);  % At each timestep, the torque of each motor needs to be specified
torque_history = repmat(torque_scale*sin(torque_omega*(1:num_s)),N,1);

q0 = zeros(N,1);
% q0(1,1) = pi/6;
% q0(2,1) = pi/6;
% q0(3,1) = pi/10;
% q0(4,1) = pi/10;
% q0(5,1) = -pi/6;

qd0 = zeros(N,1);

sim = new_sim('steps', num_s, 'sim_time', t_sim, 'chain', chain_single, ...
    'torques', torque_history, 'q0', q0, 'qd0', qd0);

tic;
sim = run_sim(sim, num_s);
toc
draw_sim(sim,varargin{:});
success = 1;

elseif ((sim_num == 3) || (~sim_num))
%% Multiple Modules
close all

global sim

chain_single = [
    new_link('HT1','rotate', rotY(pi/2));
    new_link('HT1');
    ];

N = size(chain_single, 1);

t_sim = 10;    % Simulate for t_sim [s]
num_s = 1000;  % Use this many timesteps

torque_history = zeros(N,num_s);  % At each timestep, the torque of each motor needs to be specified
q0 = zeros(N,1);
q0(1,1) = 0;
q0(2,1) = pi/6;

qd0 = zeros(N,1);

sim = new_sim('steps', num_s, 'sim_time', t_sim, 'chain', chain_single, ...
    'torques', torque_history, 'q0', q0, 'qd0', qd0);

tic;
sim = run_sim(sim, num_s);
toc
draw_sim(sim,varargin{:});
success = 1;
end
%% 
end