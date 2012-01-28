function sim = step_sim(sim)
% Takes a simulator structure and steps it forward
% in time by one step while updating all necessary
% information in the sim structure.
%
% ARGUMENTS
%  sim - simulation structure fully filled out and ready to go
%
% RETURNS
%  sim - a simulator structure with one more step in time filled out
% NOTE: Right now the architecture of this simulator is asinine and
%       not understandable by anyone who didn't write it.  This will
%       change, be patient.

dt = sim.dt;
s_cur = sim.s;
chain = sim.chain;

step_func = @(s,u)(state_rate(chain, s, u));
N = length(sim.chain);

next_state = rk4step([sim.q(:,s_cur); sim.qd(:,s_cur)], sim.T(:,s_cur), step_func, dt);

% Fill out the next state
sim.s = s_cur + 1;
sim.q(:,sim.s) = next_state(1:N);
sim.qd(:,sim.s) = next_state(N+1:end);

end