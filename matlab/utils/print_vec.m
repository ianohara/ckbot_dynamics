function string = print_vec(v)
% Prints a column vector as compactly as possible.
%
% ARGUMENTS:
%  v - vector to pretty print
% RETURNS
%  success = 1 for success, 0 for failure
%
% NOTES:
%  Only supports column vectors for now.

name = inputname(1);
v_str = sprintf('%s=', name);

rows = size(v,1);
cols = size(v,2);

if (cols ~= 1)
    error('only support column vectors.')
end

name_line = ceil(rows/2);

for i=1:rows
    if (i==name_line)
        row_beg = v_str;
    else
        row_beg = repmat('|', 1, length(v_str));
    end
    fprintf(strcat(row_beg, '%2.3f\n'), v(i));
end

success = 1;
end