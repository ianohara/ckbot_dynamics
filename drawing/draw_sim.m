function success = draw_sim(sim, varargin)
% Draws a fully calculated simulation in steps and plots
% the joint and joint velocity time histories.
%
% ARGUMENTS
%  sim - A completely calculated simulator structure
%  pause_mult - Multiple of actual simulator dt to pause between each plot.
%               NOTE: the plotting is normally CPU bound anyway
%  to_plot - Chain links to plot in the time history plot
%
% RETURNS
%  success - (Code for: nothing useful)
%
% TODO:
%   When plotting joint angles, why doesn't matlab change the line colors?
%   Add Sensible legends/ways to distinguish which module is which
%   Add projection of CMs onto ground plane/other fancy plotting

% Settings
props = {};
props.pause_mult = 1;
props.to_plot = 1:size(sim.chain,1);
props.draw_cks = 0;
props.real_time = 0;
props.draw_plots = 1;
props.just_vectors = 0;

%%% Parse any ARGUMENTS %%%
i = 1;
while (i<nargin)
    switch(varargin{i})
        case 'pause_mult'
            if (i == nargin)
                error('Must supply text field as argument following "pause_mult"');
            end
            props.pause_mult = varargin{i+1};
            i=i+1;
        case 'to_plot'
            if (i == nargin)
                error('Must supply text field as argument following "to_plot"');
            end
            props.to_plot = varargin{i+1};
            i=i+1;
        case 'draw_cks'
            props.draw_cks = 1;
            props.real_time = 1;
        case 'real_time'
            props.real_time = 1;
        case 'just_vectors'
            props.just_vectors = 1;
        otherwise
            fprintf('Unknown option: %s',varargin{i});
    end
    i = i+1;
end

close all;

dt = sim.dt;
dt_draw = dt*props.pause_mult;
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
    if (props.draw_cks)
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
        if (props.just_vectors)
            draw_chain(chain, 'just_vectors');
            draw_geom_vecs(chain);
        else
            draw_chain(chain);
        end
    end
    if (props.real_time)
        drawnow();
        pause(dt_draw);
    end
    if ((props.draw_plots) && props.real_time)
        figure(plot_fig);
        for n=1:size(props.to_plot)
            subplot(211);
            plot(i*dt, sim.q(props.to_plot(n),i), 'o');
            subplot(212);
            plot(i*dt, sim.qd(props.to_plot(n),i), 'o');
        end
    end
end

if  ((props.draw_plots) && (~props.real_time))
    figure(plot_fig)
    st = 1:steps;
    for n = 1:length(props.to_plot)
        fprintf('Plotting link number %d...\n',n);
        subplot(211);
        plot(st*dt, sim.q(props.to_plot(n),:),'LineWidth',2);
        hold on;
        subplot(212);
        plot(st*dt, sim.qd(props.to_plot(n),:),'LineWidth',2);
        hold on;
    end
end
success = 1;
end