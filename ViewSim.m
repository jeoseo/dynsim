function ViewSim(t,js)
    %Setup the figure and start drawing
    figure();
    set(gcf,'color','w');
    set(gcf,'position',[0 0 1400 900]);
    grid on;
    axis equal;
    xlim([-4 4]);
    ylim([-4 4]);
    zlim([-4 4]);
    view(30,30);
    for i=1:20:size(js,2) %specify framerate
        gth=ComputeFK(js(1:end/2,i));
        DrawRobot(gth,0.2);
        title(t(i));
        drawnow;
    end
end