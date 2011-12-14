function h = draw_triad(r,R,s,frame_style)
%%%%%
%  Draws a reference frame triad whose origin is located at 
%  r (3x1 position vector) and whose orientation is defined 
%  by the 3x3 rotation matrix R.
%
switch(frame_style)
    case 'rgb'
        ac = {'r' 'g' 'b'};
        lw = 1.5;
    case 'bw'
        ac = {'k' 'k' 'k'};
        lw = 1.0;
    case 'xy'
        ac = {'k' 'k' 'k'};
        lw = 1.5;
    case 'myc'
        ac = {'m' 'y' 'c'};
        lw = 1.5;
    otherwise
        ac = {'k' 'k' 'k'};
        lw = 1.0;
end
r = reshape(r,3,1);

xaxis = [r r + s * R(:,1)];
yaxis = [r r + s * R(:,2)];
zaxis = [r r + s * R(:,3)];

h=plot3(xaxis(1,:),xaxis(2,:),xaxis(3,:),ac{1},...
    yaxis(1,:),yaxis(2,:),yaxis(3,:),ac{2},...
    zaxis(1,:),zaxis(2,:),zaxis(3,:),ac{3},...
    'LineWidth',lw);

xdir = (xaxis(:,2)-xaxis(:,1))./2;
ydir = (yaxis(:,2)-yaxis(:,1))./2;
zdir = (zaxis(:,2)-zaxis(:,1))./2;

xaxis = xaxis(:,1)+xdir;
yaxis = yaxis(:,1)+ydir;
zaxis = zaxis(:,1)+zdir;

% Draw cones at each tip
cone_x =coneplot(xaxis(1,1), xaxis(2,1), xaxis(3,1), xdir(1), xdir(2), xdir(3),'color', ac{1},'nointerp');
cone_y =coneplot(yaxis(1,1), yaxis(2,1), yaxis(3,1), ydir(1), ydir(2), ydir(3),'color', ac{2},'nointerp');

cone_z =coneplot(zaxis(1,1), zaxis(2,1), zaxis(3,1), zdir(1), zdir(2), zdir(3),'color', ac{3},'nointerp');

set(cone_x, 'FaceColor', ac{1});
set(cone_x, 'EdgeColor', ac{1});

set(cone_y, 'FaceColor', ac{2});
set(cone_y, 'EdgeColor', ac{2});

set(cone_z, 'FaceColor', ac{3});
set(cone_z, 'EdgeColor', ac{3});


if(strcmp(frame_style,'xy'))
    xpos = 1.01*xaxis(:,2);
    text(xpos(1),xpos(2),xpos(3),'x','FontSize',12);
    ypos = 1.01*yaxis(:,2);
    text(ypos(1),ypos(2),ypos(3),'y','FontSize',12);
elseif(ac{1} == 'k') %if black triad show z
    zpos = 1.01*zaxis(:,2);
    text(zpos(1),zpos(2),zpos(3),'z','FontSize',12);
end



end