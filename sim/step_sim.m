function [q_next, qd_next] = step_sim(chain, q, qd, T, dt)
% Takes a simulator structure and steps it forward
% in time by one step while updating all necessary
% information in the sim structure.
%
% ARGUMENTS
% 
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

step_func = @(s,u)(state_rate(chain, s, u));
N = size(chain,1);

next_state = rk4Step([q; qd], T, step_func, dt);

q_next = next_state(1:N);
qd_next = next_state(N+1:end);

end