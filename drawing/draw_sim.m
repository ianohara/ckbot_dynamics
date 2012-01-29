function success = draw_sim(sim, pause_mult, to_plot)
% Draws a fully calculated simulation in steps and plots
% the joint and joint velocity time histories.
%
% ARGUMENTS
%  sim - A completely calculated simulator structure
%  pause_mult - Multiple of actual simulator dt to pause between each plot.
%               NOTE: the plotting is normally CPU bound anyway
%  to_plot - Chain links to plot in the time history plot
%

% Settings (To be replaced with Varargin parsing soon)
real_time = 1;
draw_cks = 1;
draw_plots = 1;
just_vectors = 1;


close all;

dt = sim.dt;
dt_draw = dt*pause_mult;
steps = sim.s;
chain = sim.chain;

draw_fig = figure();
hold on;
grid on;
axis equal;
title('Visualization of CKBot System Evolution with Time','FontSize', 14);
xlabel('X [m]','FontSize',14);
ylabel('Y [m]', 'FontSize', 14);
zlabel('Z [m]', 'FontSize', 14);
view(45,45);

plot_fig = figure();
subplot(211);
grid on;
hold on;
title('Time History of CKBot System','FontSize', 14);
xlabel('Time [s]','FontSize', 14);
ylabel('Joint Angle [rad]','FontSize',14);

subplot(212);
grid on;
hold on;
xlabel('Time [s]', 'FontSize', 14);
ylabel('Joint Angular Velocity [rad/s]', 'FontSize', 14);

for i=1:steps
    if (draw_cks)
        figure(draw_fig);
        clf;
        hold on;
        grid on;
        axis equal;
        title('Visualization of CKBot System Evolution with Time','FontSize', 14);
        xlabel('X [m]','FontSize',14);
        ylabel('Y [m]', 'FontSize', 14);
        zlabel('Z [m]', 'FontSize', 14);
        view(45,45);
        fprintf('Drawing step %d (Time = %f)\n', i, i*dt);
        chain = propogate_angles_and_rates(chain, sim.q(:,i), sim.qd(:,i));
        if (just_vectors)
            draw_chain(chain, 'just_vectors');
            draw_geom_vecs(chain);
        else
            draw_chain(chain);
        end
    end
    if (real_time)
        drawnow();
        pause(dt_draw);
    end
    if ((draw_plots) && real_time)
        figure(plot_fig);
        for n=1:size(to_plot)
            subplot(211);
            plot(i*dt, sim.q(n,i), 'o');
            subplot(212);
            plot(i*dt, sim.qd(n,i), 'o');
        end
    end
end

if  ((draw_plots) && (~real_time))
    figure(plot_fig)
    st = 1:steps;
    for n = 1:length(to_plot)
        subplot(211);
        plot(st*dt, sim.q(n,:),'LineWidth',2);
        subplot(212);
        plot(st*dt, sim.qd(n,:),'LineWidth',2);
    end
end

end