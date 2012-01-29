function chain = propogate_angles_and_rates(chain, q, qd)
% Update the angles and rates of a chain
%
%

for i = 1:size(chain,1)
   chain(i).q = q(i);
   chain(i).qd = qd(i);
end
end