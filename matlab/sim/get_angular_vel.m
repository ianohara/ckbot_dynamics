function omega = get_angular_vel(chain, n, qd)
% Returns the angular velocity of the nth link in the chain
% in the inertial frame
%
% ARGUMENTS
%  chain - an array of N links where chain(1) is the base, 
%          chain(N) is the tip
%  n - link (1 <= n <= N) for which to return omega in the inertial frame
%
% RETURNS
%  omega - angular velocity of link n in the inertial frame
%
% TODO:
%  
%

omega = [0; 0; 0];
R = chain(1).init_rotation;

for i=1:n-1
    omega = omega + R*[0; 0; qd];
    R = R*chain(i).R_jts;
end

end