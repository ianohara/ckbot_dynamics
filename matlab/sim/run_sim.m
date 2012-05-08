function sim = run_sim(sim, steps)
% Step a simulation through a certain number of steps.
% 
% ARGUMENTS
%   sim - Simulator structure to run
%   steps - Number of steps to run the simulator for
% RETURNS
%   sim - Simulator structure with sim.q and sim.qd
%         filled in.
%

if (length(sim.dt) == 1)
    sim.dt = repmat(sim.dt, 1, steps);
elseif (length(sim.dt) ~= steps)
    error('sim.dt must either be a vector of length steps or a scalar. (ie: specify a timestep for each step, or specify a single timestep to use for all steps).');
end

for i = 1:steps-1
    [q_n,qd_n] = step_sim(sim.chain, sim.q(:,i), sim.qd(:,i), sim.T(:,i), sim.dt(i), sim.integrator);
    sim.q(:,i+1) = q_n;
    sim.qd(:,i+1) = qd_n;
    sim.s = sim.s+1;
end

end