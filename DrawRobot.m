%draws the robot given computed FK, in terms of a homogenous transform
%Assumes the base frame is at the origin
%gth is 4x4xDOFxk, where k=1 represents the end of each link (and thus we 
%draw a line in between them, k=2 is the COM, and k>2 can be whatever.
%scale handles the size of the coordinate axes produced
function DrawRobot(gth,scale)
     to_delete=[findobj(gcf,'Type','line');findobj(gcf,'Type','quiver')];
     delete(to_delete); %deletes just the lines and quivers in the plot, aka the previous robot model
     hold on;
     home=[0;0;0];%base of the line segment
     for i=1:size(gth,3)
         t_home=[gth(1,4,i,1);gth(2,4,i,1);gth(3,4,i,1)];
         line([home(1),t_home(1)],[home(2),t_home(2)],[home(3),t_home(3)],'Color','black');
         home=t_home;
         for j=1:size(gth,4)
             DrawPose(gth(:,:,i,j),scale);
         end
     end
     hold off;
end