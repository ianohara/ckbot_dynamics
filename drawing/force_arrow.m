function success = draw_arrow(start, stop)
% ARGUMENTS
%  start - 1x3 vector where the tail of the force arrow should be located
%  stop  - 1x3 vector where the head of the arrow should be located
% RETURNS
%  success - 1 for success, 0 for failure
% TODO
%  * Add color support
%


arrow = stop-start;
cone_start = start + arrow/2;
coneplot(cone_start(1),cone_start(2),cone_start(3),arrow(1),arrow(2),arrow(3),'nointerp');
line([start(1) stop(1)], [start(2) stop(2)], [start(3) stop(3)],'LineWidth', 5, 'Color', 'k');
end