function link = new_link(type, varargin)
% Creates a new link to add to a serial chain CKBot simulation
% 
% ARGUMENTS
%  
%  type - [text] - Specifies the type of link to create
%  
% RETURNS
%  link - [structure]

props = get_body_params(type);

% Joint variables
props.q = 0;
props.qd = 0;

%%% Parse any ARGUMENTS %%%
i = 1;
while (i<nargin)
    switch(varargin{i})
        case 'q'
            if (i == nargin)
                error('Must supply text field as argument following "q"');
            end
            props.q = varargin{i+1};
            i=i+1;
        case 'qd'
            if (i == nargin)
                error('Must supply text field as argument following "qd"');
            end
            props.qd = varargin{i+1};
            i=i+1;
        otherwise
            fprintf('Unknown option: "%s"\n',varargin{i});
    end
    i = i+1;
end

link = {};

link.q = props.q; % Angle of rotation about join wrt the previous link

% Rate of rotation about previous link's joint 
% (note: this implies link i's motor is at its base and rotates around
% i-1's joint)
link.qd = props.qd; 

link.forward_joint_axis = props.joint_axis;
link.r_im1 = props.r_back; % Vector in link coords to the joint of previous link
link.r_ip1 = props.r_forward; % Vector in link coords to the joint of next link
link.I_cm = props.I_cm;
link.R_jts = props.R_jts;
link.m = props.mass;
link.draw = props.draw_fun;

end