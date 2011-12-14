function success = print_if_verbose(p, msg, varargin)
% ARGUMENTS
%  p struct with p.verbose defined.  p.verbose > 0 means verbose is on
%  msg - string for fprintf
%  varargin - variables for fprinft (ie: fprinft(msg,varargin))
% RETURNS
%  success - 1 if the fprintf worked fine.  0 if it failed
% TODO
%
if (p.verbose > 0)
    % Get the name of the function that we're reporting from
    [stack, index] = dbstack();
    caller_name = stack(2).name;
    line_number = stack(2).line;
    msg = sprintf('%s (ln %d): %s', caller_name, line_number, msg);
    if (nargin > 2)
        fprintf(msg);
    else
        fprintf(msg);
    end
end
success = 1;
end