function success = draw_geom_vecs(chain)
%  Draws the forward link and backward link vectors from each link's
%  center of mass.
%
success = 0;

r_base = [0; 0; 0];  % First chain link starts at the origin
R = eye(3);
for i = 1:length(chain)
    R = R*rotZ(chain(i).q)*chain(i).R_jts;  % From i's coordinates to inertial
    r_cm = r_base - R*(chain(i).r_im1);

    draw_arrow(r_cm, r_base); % Backward link arrow
    
    r_base = r_cm + R*(chain(i).r_ip1);
    draw_arrow(r_cm, r_base); % Forward link now that r_base is updated
end
end