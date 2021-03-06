function chain = draw_environment(sim,s)
%
% Takes a simulation structure and draws it for you.  By default,
% this just draws the initial configuration, but options can
% be used to make it draw different steps (if they exist) or to
% plot the motion of a full simulation.
%
% ARUMENTS:
%  sim - simulation structure
%  s - step of simulation to draw
%
% RETURNS:
%  success - 1 if successful, 0 if not

sets = {};
sets.draw_geom = 1;
sets.draw_vels = 1;
sets.draw_geom_vecs = 1;

chain = sim.chain;

N = length(chain);
for i=1:N
   chain(i).q =  sim.q(i,s);
   chain(i).qd = sim.qd(i,s);
end

close all;
figure;
hold on;
grid on;
axis equal;
xlabel('X [m]','FontSize',14);
ylabel('Y [m]','FontSize',14);
zlabel('Z [m]','FontSize',14);
view(45,45);

if (sets.draw_geom)
    draw_chain(chain);
end
if (sets.draw_vels)
    draw_lin_vels(chain);
end
if (sets.draw_geom_vecs)
    draw_geom_vecs(chain);
end
end