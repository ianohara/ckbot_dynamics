function success = draw_chain(chain, varargin)
%  Draws a chain of connected CKBot links in the current figure
%  
%  ARGUMENTS:
%    chain - vector of link structures in the order of their linking
%
%  RETURNS:
%    success - 1 for success, 0 for failure
success = 0;

r_base = [0; 0; 0];  % First chain link starts at the origin

R = chain(1).init_rotation; 

for i = 1:length(chain)
    
    R = R*rotZ(chain(i).q)*chain(i).R_jts;  % From i's coordinates to inertial
    r_cm = r_base - R*(chain(i).r_im1);
    
    chain(i).draw(r_cm,R);
    draw_triad(r_cm, R, 1, 'rgb');
    r_base = r_cm + R*(chain(i).r_ip1);
    draw_triad(r_base, R, 1, 'myc');
end
success = 1;
end