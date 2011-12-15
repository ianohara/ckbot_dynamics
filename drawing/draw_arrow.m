function success = draw_arrow(start, stop)
% ARGUMENTS
%  start - 1x3 vector where the tail of the force arrow should be located
%  stop  - 1x3 vector where the head of the arrow should be located
% RETURNS
%  success - 1 for success, 0 for failure
% TODO
%  * Add color support
%

success = 0; 
use_cone = 1;

arrow = stop-start;
if (norm(arrow) < eps)
    success = 1;
    return;
end
if (use_cone)
    cone_fraction = 0.7;
    cone_start = start + arrow*cone_fraction;
    cone_len = norm(stop-cone_start);
    cone_dir = (stop-start)/norm(stop-start);
    coneplot(cone_start(1),cone_start(2),cone_start(3),cone_len*cone_dir(1), cone_len*cone_dir(2), cone_len*cone_dir(3),'nointerp');
end
line([start(1) stop(1)], [start(2) stop(2)], [start(3) stop(3)],'LineWidth', 3, 'Color', 'k');

success = 1;
end