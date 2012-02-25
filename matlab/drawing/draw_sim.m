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
%   
%   Add comments - Right now this is a giant wall of code


% Settings
props = {};
props.pause_mult = 1;
props.to_plot = 1:size(sim.chain,1);
props.draw_cks = 0;
props.real_time = 0;
props.draw_plots = 1;
props.just_vectors = 0;
props.draw_torque = 0;
props.start_time = 0;
props.one_figure = 0;

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
        case 'start_time'
            if (i==nargin)
                error('Must supply text field as argument following "start_time".');
            end
            props.start_time = varargin{i+1};
            i=i+1;
        case 'draw_cks'
            props.draw_cks = 1;
            props.real_time = 1;
        case 'draw_torque'
            props.draw_torque = 1;
        case 'real_time'
            props.real_time = 1;
        case 'just_vectors'
            props.just_vectors = 1;
        case 'one_figure'
            props.one_figure = 1;
        case 'make_movie'
            props.make_movie = 1;
        otherwise
            fprintf('Unknown option: %s',varargin{i});
    end
    i = i+1;
end

fprintf('Starting at time %e...\n', props.start_time);

close all;

dt = sim.dt;
dt_draw = dt*props.pause_mult;
steps = sim.s;
chain = sim.chain;

start_step = floor(props.start_time/dt)+1;

% Subplots
draw_fig = figure();
if (props.one_figure)
   plot_fig = draw_fig;
   torque_fig = draw_fig;
   joint_vel_sp = 224;
   joint_ang_sp = 223;
   torque_sp = 222;
   ck_sp = 221;
else
    plot_fig = figure();
    torque_fig = figure();
    joint_vel_sp = 212;
    joint_ang_sp = 211;
    torque_sp = 111;
    ck_sp = 111;
end



figure(draw_fig);
subplot(ck_sp);
hold on;
grid on;
axis equal;
title('Visualization of CKBot System Evolution with Time','FontSize', 14);
xlabel('X [m]','FontSize',14);
ylabel('Y [m]', 'FontSize', 14);
zlabel('Z [m]', 'FontSize', 14);
view(45,45);

if (props.make_movie)
   set(gcf, 'Position', get(0, 'ScreenSize')); % Maximize figure.
   movie_dir = 'movies/';
   movie_name = datestr(now,'mm-dd@HH-MM');
end

figure(plot_fig);
subplot(joint_ang_sp);
grid on;
hold on;
title('Time History of CKBot System','FontSize', 14);
xlabel('Time [s]','FontSize', 14);
ylabel('Joint Angle [rad]','FontSize',14);

subplot(joint_vel_sp);
grid on;
hold on;
xlabel('Time [s]', 'FontSize', 14);
ylabel('Joint Angular Velocity [rad/s]', 'FontSize', 14);

figure(torque_fig);
subplot(torque_sp);
grid on;
hold on;
title('Time History of CKBot System','FontSize', 14);
xlabel('Time [s]','FontSize', 14);
ylabel('Joint Torque [Nm]','FontSize',14);
st = 1:steps;
plot(st*dt, sim.T(props.to_plot,:),'LineWidth', 2);
grid on;
hold on;

if  (props.draw_plots)
    figure(plot_fig)
    st = 1:steps;
    subplot(joint_ang_sp);
    plot(st*dt, sim.q(props.to_plot,:),'LineWidth',2);
    hold on;
    subplot(joint_vel_sp);
    plot(st*dt, sim.qd(props.to_plot,:),'LineWidth',2);
    hold on;
    legend_strs = {};
    for i=1:length(props.to_plot)
       fprintf('Link %d\n', props.to_plot(i));
       legend_strs{end+1} = sprintf('Link %d', props.to_plot(i));
    end
    legend(legend_strs);
end

figure(plot_fig);
subplot(joint_ang_sp);
cur_axis = axis();
t_line1 = line([0.0 0.0], cur_axis(3:4), 'LineStyle', '-', 'LineWidth',2,'Color','k');
subplot(joint_vel_sp);
cur_axis = axis();
t_line2 = line([0.0 0.0], cur_axis(3:4),'LineStyle', '-', 'LineWidth',2,'Color','k');

figure(torque_fig);
subplot(torque_sp);
cur_axis = axis();
t_line3 = line([0.0 0.0], cur_axis(3:4), 'LineStyle', '-', 'LineWidth', 2, 'Color', 'k');

ck_axis = 0;

for i=start_step:steps
    if (props.draw_cks)
        figure(draw_fig);
        subplot(ck_sp);
        [az, el] = view();
        cla;
        hold on;
        grid on;
        title('Visualization of CKBot System Evolution with Time','FontSize', 14);
        xlabel('X [m]','FontSize',14);
        ylabel('Y [m]', 'FontSize', 14);
        zlabel('Z [m]', 'FontSize', 14);
        view(az, el);
        chain = propogate_angles_and_rates(chain, sim.q(:,i), sim.qd(:,i));
        if (props.just_vectors)
            draw_chain(chain, 'just_vectors');
            draw_geom_vecs(chain);
        else
            draw_chain(chain, 'link_nums');
        end
        if (ck_axis)
            axis(ck_axis);
        else
            axis equal;
            cur_ax = axis();
            %ck_axis = [min(cur_ax) max(cur_ax) min(cur_ax) max(cur_ax) min(cur_ax) max(cur_ax)];
            offset = 0.4;
            ck_axis = [cur_ax(1)-offset cur_ax(2)+offset cur_ax(3)-offset cur_ax(4)+offset cur_ax(5)-offset cur_ax(6)+offset];
        end
    end
    if (props.real_time)
        figure(draw_fig);
        subplot(ck_sp);
        set(t_line1,'XData', [i*dt, i*dt]);
        set(t_line2, 'XData', [i*dt, i*dt]);
        set(t_line3, 'XData', [i*dt, i*dt]);
        drawnow();
        if (props.make_movie)
            print(draw_fig, '-dpng', sprintf('%s%05d_%s_Frame.png', movie_dir, i,movie_name)); 
        end
        pause(dt_draw);
    end
end
success = 1;

if  (props.make_movie)
    close all
end

end
