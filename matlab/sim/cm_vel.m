function vel = cm_vel(chain, link_no)

vel = [0;0;0];

for i = link_no:-1:1
   if (i == link_no)
       r_cur = -chain(i).r_im1;
   else
       r_cur = -chain(i).r_im1 + chain(i).r_ip1;
   end
   omega_this = chain(i).R_jts*[0;0;chain(i).qd]; % Relative angular vel in this link's coordinate frame
   v_this_from_base = cross(omega_this, r_cur) + vel;
   vel = rotZ(chain(i).q)'*chain(i).R_jts'*v_this_from_base; % As seen from link before this one
end

end