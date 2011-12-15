function success = draw_chain(chain, varargin)
%  Draws a chain of connected CKBot links
%  
%  ARGUMENTS:
%    chain - vector of link structures in the order of their linking
%
%  RETURNS:
%    success - 1 for success, 0 for failure
success = 0;

r_base = [0; 0; 0];  % First chain link starts at the origin

% Check if there's an initial link inertial rotation, otherwise use the
% default.  Note that this means the 'init_rotation' field is ignored on
% links that aren't the first in their chain.
if (isfield(chain(1), 'init_rotation'))
    R = chain(1).init_rotation; 
else
    R = eye(3);
end

for i = 1:length(chain)
    
    R = R*rotZ(chain(i).q)*chain(i).R_jts;  % From i's coordinates to inertial
    r_cm = r_base - R*(chain(i).r_im1);
    
    chain(i).draw(r_cm,R);
    draw_triad(r_cm, R, 1, 'rgb');
    r_base = r_cm + R*(chain(i).r_ip1);
    draw_triad(r_base, R, 1, 'myc');
end
end