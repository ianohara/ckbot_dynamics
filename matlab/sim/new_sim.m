function sim = new_sim(varargin)
% Create a new simulation struct that we can pass to our simulator
%
% ARGUMENTS:
%   See varargin parsing below.  NOTE: all options are 'key', 'value'
%   pair style.  
%
% RETURNS:
%
% TODO: Documentation and input checking

if (mod(nargin, 2) ~= 0)
    error('Number of arguments must be even (''key'', ''value'' pairs)');
end

sim = {};

sim.integrator = @euler; % First order euler as default integrator

num_s = NaN;
t_sim = NaN;
t = NaN;
dt = NaN;

% Note: All options are 'key', value options.  
for i = 1:2:nargin-1
   opt = varargin{i};
   val = varargin{i+1};
   switch opt
       case 'steps'
           if (isnumeric(val))
               num_s = val;
           else
               error('''steps'' value must be numeric.');
           end
       case 'sim_time'
           if (isnumeric(val))
               t_sim = val;
           else
               error('''sim_time'' value must be numeric.');
           end
       case 'chain'
           sim.chain = val;
       case 'torques'
           sim.T = val;
       case 'q0'
           q0 = val;
       case 'qd0'
           qd0 = val;
       case 'integrator';
           sim.integrator = val;
       case 'dt'
           dt = val;
       otherwise
           error('Unknown option: %s', opt);
   end
end

N = size(sim.chain,1);

if (isnan(dt))
    dt = t_sim/num_s;
    t = 0:dt:(t_sim-dt);
elseif (length(dt) == 1)
    t = 0:dt:(t_sim-dt);
else
    t = zeros(1,length(dt));
    for i = 1:length(dt)-1
       t(i+1) = t(i) + dt(i);
    end
end

sim.dt = dt;

sim.t = t';

if (size(sim.t,1) ~= num_s)
    error('Number of time steps not matching requested number of steps (%d vs %d)', size(sim.t,1), num_s);
end

% Initialize all of the data structures since now we know how big they all
% need to be
sim.q = NaN(N, num_s);
sim.qd = NaN(N, num_s);
sim.qdd = NaN(N, num_s);
sim.p = NaN(6*N, 6*N, num_s);
sim.z = NaN(6*N, num_s);
sim.s_vars = {};
sim.s_vars.G = NaN(6*N,1);
sim.s_vars.mu = NaN(N,1);
sim.s_vars.a = zeros(6*N,1);

% Set the initial conditions
if (size(q0,1) ~= N)
    error('Initial joint angle vector has a length that is not the same as the number of bodies in the chain.');
end
if (size(qd0,1) ~= N)
    error('Initial joint velocity vector has a length that is not the same as the number of bodies in the chain.');
end

for i=1:N
   sim.chain(i).q = q0(i);
   sim.chain(i).qd = qd0(i);
end

sim.q(:,1) = q0;
sim.qd(:,1) = qd0;

sim.s = 1;

end