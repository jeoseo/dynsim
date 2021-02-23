%Given a single 4x4 homegenous transform g, draws a pose at the given point
%for the figure  with the coordinate axes being drawn
%with the the length specified by scale

function DrawPose(g,scale)
    hold on; %in case it wasn't already
     %find direction of coord axes
    x=g(1:3,1:3)*[1;0;0]*scale;
    y=g(1:3,1:3)*[0;1;0]*scale;
    z=g(1:3,1:3)*[0;0;1]*scale;
    quiver3(g(1,4),g(2,4),g(3,4),x(1),x(2),x(3),0,'r');
    quiver3(g(1,4),g(2,4),g(3,4),y(1),y(2),y(3),0,'g');
    quiver3(g(1,4),g(2,4),g(3,4),z(1),z(2),z(3),0,'b');
    
end