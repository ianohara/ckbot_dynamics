function s2 = euler(s1, u, fh, dt)
% Euler integration with fixed time step.
s2 = s1 + fh(s1, u)*dt;
end