function success = debug_msg(string)
    success = 0;
    if (exist('DEBUG_MSG'))
        fprintf(string);
        success = 1;
    end
end