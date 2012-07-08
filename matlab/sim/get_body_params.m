function props = get_body_params(type)
% Any function that needs the geometric parameters and inertial properties
%  of each rigid body type can just call this function and get it here.
%
% ARGUMENTS
%  type - string - A rigid body type.  Current choices are:
%                    HT1 - Head-Tail-1 (Joint axes parallel)
%                    HT2 - Head-Tail-2 (Joint axes at 90deg)
%

% Universal geometric and inertial parameters
props = {};
props.width = 0.2;
props.head_len = props.width;
props.mass = 0.1;
props.joint_axis = [0;0;1];
props.tail_len = props.width;
props.tail_width = props.width/2;
props.tail_radius = props.tail_width/2;
props.joint_mat = [0,0,1,0,0,0]; % Defn in JPL paper.
props.damping = 1.0;  % Joint damping [N * s * m]

% Drawing Options
props.transparency = 0.6;
props.face_color = 'k';
props.edge_color = 'k';

% Link Type Specific Information
%
% HT1 - Head-Tail 1
% Rotational axes of joints are parallel (ie: Planar motion of links)
%
% HT2 - Head-Tail 2
% Rotational axes are at 90deg angle
%
% HT_head_cap
%  A UBar (head) at the end of a chain
%
% HT_tail_cap
%  A Motor mount (tail) at the end of a chain
%

if (strcmp(type, 'HT1'))
    props.r_back = [-props.head_len/2; 0; 0];
    props.r_forward = [props.head_len/2; 0; 0];
    props.I_cm = eye(3);
    props.R_jts = eye(3);
    props.draw_fun = @draw_head_tail_1;
elseif (strcmp(type, 'HT2'))
    props.r_back = [-props.head_len/2; 0; 0];
    props.r_forward = [props.head_len/2; 0; 0];
    props.I_cm = eye(3);
    props.R_jts = rotX(-pi/2);
    props.draw_fun = @draw_head_tail_2;
elseif (strcmp(type, 'HT_head_cap'))
    props.r_back = [0;0;0];
    props.r_forward = [0;0;0];
    props.I_cm = eye(3);
    props.R_jts = eye(3);
    props.draw_fun = @draw_head_cap;
elseif (strcmp(type, 'HT_tail_cap'))
    props.r_back = [0;0;0];
    props.r_forward = [props.head_len; 0; 0];
    props.I_cm = eye(3);
    props.R_jts = eye(3);
    props.draw_fun = @draw_tail_cap;
else
    error('Unknown body type: %s', type);
end

end