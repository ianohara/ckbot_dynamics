function success = draw_chain(chain, varargin)
%  Draws a chain of connected CKBot links in the current figure
%  
%  ARGUMENTS:
%    chain - vector of link structures in the order of their linking\
%    varargin - See below
%
%  RETURNS:
%    success - 1 for success, 0 for failure
success = 0;

% Hackery for checking varargins.  This works pretty well
% for flag style options though.

% Don't plot the patch derived CKBots (They're slow as hell, so this is
% nice for "real time" plotting)
if (sum(ismember(varargin, 'just_vectors')) > 0)
    just_vectors = 1;
else
    just_vectors = 0;
end

if (sum(ismember(varargin, 'link_nums')) > 0)
    draw_nums = 1;
else
    draw_nums = 0;
end

% For using draw_chain as a standalone function.
if (sum(ismember(varargin, 'need_figure')) > 0)
    fprintf('Initializing a figure for you...\n');
    figure;
    hold on;
    grid on;
    axis equal;
    xlabel('X [m]','FontSize',14);
    ylabel('Y [m]','FontSize',14);
    zlabel('Z [m]','FontSize',14);
    view(45,45);
end

r_base = [0; 0; 0];  % First chain link starts at the origin

R = chain(1).init_rotation; 

% Nothing significant about using this, it just is a nice way to scale the
% triads as the module dimensions change.
triad_scale = norm(2*chain(1).r_im1); 

for i = 1:length(chain)
    R = R*rotZ(chain(i).q)*chain(i).R_jts;  % From i's coordinates to inertial
    r_cm = r_base - R*(chain(i).r_im1);
    if (~just_vectors)
        chain(i).draw(r_cm,R);
    end
    if (draw_nums)
       text(r_cm(1), r_cm(2), r_cm(3), sprintf('%d', i), 'FontSize', 14,'Color', 'w');
    end
    draw_triad(r_cm, R, triad_scale, 'rgb');
    r_base = r_cm + R*(chain(i).r_ip1);
    draw_triad(r_base, R, triad_scale, 'myc');
    
end
success = 1;
end