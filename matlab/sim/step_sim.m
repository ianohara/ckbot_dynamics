function [q_next, qd_next] = step_sim(chain, q, qd, T, dt, varargin)
% Takes a simulator structure and steps it forward
% in time by one step while updating all necessary
% information in the sim structure.
%
% ARGUMENTS
%    chain - chain of ckbot modules
%    q - Current angles of modules
%    qd - Current joint speeds of modules
%    T - Current joint Torques of modules
%    dt - timestep to step forward
%    varargin - If an additional argument is given, it is assumed to be
%               a function handle for a non-default integrator
%
% RETURNS
%  sim - a simulator structure with one more step in time filled out
% NOTE: Right now the architecture of this simulator is asinine and
%       not understandable by anyone who didn't write it.  This will
%       change, be patient.
%
% TODO:
%   Change this to take a function handle for the integrator as well.
%

if (nargin > 5)
    integrator = varargin{1};
else
    integrator = @rk4Step;
end

step_func = @(s,u)(state_rate(chain, s, u));
N = size(chain,1);

%next_state = rk4Step([q; qd], T, step_func, dt);
next_state = integrator([q; qd], T, step_func, dt);

q_next = next_state(1:N);
qd_next = next_state(N+1:end);

end