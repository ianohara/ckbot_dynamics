function sim = sim_test(varargin)

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

chain_single(1).damping = 0.0;
chain_single(2).damping = 0.0;

chain_single(1).I_cm = 0.00083333*eye(3);
chain_single(2).I_cm = 0.00083333*eye(3);

N = size(chain_single, 1);
t_sim = 1.5;
num_s = 300;
% DEBUG
 % dt = 0.01;
 t_sim = 0.02;
 num_s = 2;
torque_history = zeros(N, num_s);
q0 = zeros(N,1);
q0(1) = 1.57;
q0(2) = 1.57;

qd0 = zeros(N,1);
% qd0(1) = 1;
% qd0(2) = 1;

sim = new_sim('steps', num_s, 'sim_time', t_sim, 'chain', chain_single, ...
    'torques', torque_history, 'q0', q0, 'qd0', qd0, 'integrator', @rk4Step);

tic;
sim = run_sim(sim, num_s);
toc;

draw_sim(sim, varargin{:});

elseif ((sim_num == 2) || (~sim_num))
%% Multiple Modules
close all

global sim

chain_single = [
    new_link('HT1','rotate', rotY(pi/2));
    new_link('HT1');
    new_link('HT2');
    new_link('HT1');
    new_link('HT1');
    new_link('HT2');
    new_link('HT1');
    new_link('HT1');
    ];

N = size(chain_single, 1);

t_sim = 10;    % Simulate for t_sim [s]
num_s = 1000;  % Use this many timesteps

torque_scale = 1;
torque_omega = 1/20;

torque_history = zeros(N,num_s);  % At each timestep, the torque of each motor needs to be specified
%torque_history(1,:) = torque_scale*sin(torque_omega*(1:num_s));

q0 = zeros(N,1);
 q0(1,1) = 0.1;
  q0(2,1) = pi/6;
  q0(3,1) = pi/10;
  q0(4,1) = pi/10;
  q0(5,1) = -pi/6;

qd0 = zeros(N,1);

sim = new_sim('steps', num_s, 'sim_time', t_sim, 'chain', chain_single, ...
    'torques', torque_history, 'q0', q0, 'qd0', qd0, 'integrator', @euler);

tic;
sim = run_sim(sim, num_s);
toc
draw_sim(sim,varargin{:});

elseif ((sim_num == 3) || (~sim_num))
%% Multiple Modules
close all

global sim

chain_single = [
    new_link('HT1','rotate', rotY(pi/2));
    new_link('HT1');
    new_link('HT1');
    new_link('HT1');
    new_link('HT1')
    ];

N = size(chain_single, 1);

t_sim = 20;    % Simulate for t_sim [s]
num_s = 2000;  % Use this many timesteps

torque_history = zeros(N,num_s);  % At each timestep, the torque of each motor needs to be specified
q0 = zeros(N,1);
q0(1,1) = pi/2;

qd0 = zeros(N,1);

sim = new_sim('steps', num_s, 'sim_time', t_sim, 'chain', chain_single, ...
    'torques', torque_history, 'q0', q0, 'qd0', qd0);

tic;
sim = run_sim(sim, num_s);
toc
draw_sim(sim,varargin{:});
elseif ((sim_num == 4) || (~sim_num))

fprintf('Running simulation number 4:\n');
close all
global sim

chain_single = [
    new_link('HT2','rotate',rotY(pi/2));
    ];

chain_single(1).damping = 0.0;
chain_single(1).m = 12;
chain_single(1).r_im1 = [-0.5; 0; 0];
chain_single(1).r_ip1 = [0.5; 0; 0];
chain_single(1).I_cm = eye(3);

N = size(chain_single, 1);
t_sim = 2;
num_s = 200;

torque_history = zeros(N, num_s);
torque_history = 29.43*ones(N, num_s); % DEBUG
q0 = zeros(N,1);
q0(1) = 1.57;

qd0 = zeros(N,1);
qd0(1) = 0;

sim = new_sim('steps', num_s, 'sim_time', t_sim, 'chain', chain_single, ...
    'torques', torque_history, 'q0', q0, 'qd0', qd0, 'integrator', @rk4Step);

tic;
sim = run_sim(sim, num_s);
toc;

draw_sim(sim, varargin{:}); 
end
%% 
end