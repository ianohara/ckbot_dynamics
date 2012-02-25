function dsdt = state_rate(chain, s, T)
% For use with whatever integrator the simulator
% ends up using.  For example, the 4th order runge kutta
% integrator needs to be able to specify arbitrary
% dt's and states.
%
% ARGUMENTS
%  sim - simulator structure with a chain
%  s - state to use as the seed state for the integrator
%  T - motor torques input to use
% RETURNS
%  Rate of change of state for this chain at state s with inputs u
%

N = size(chain,1);

q = s(1:N);
qd = s(N+1:end);

if (length(q) ~= length(qd))
   error('Length of state vector is not twice the number of joints.'); 
end

if (size(T,1) ~= N)
   error('Length of torque input vector must be the same as the number of joints.');
end

[G,mu,a] = tip_base_step(chain, s, T);
qdd = base_tip_step(chain, s, T, G, mu, a);
dsdt = [qd; qdd];
end