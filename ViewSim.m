function ViewSim(t,js, record)
    if (record==1)
         writerObj = VideoWriter('record.avi');
         writerObj.FrameRate = 20;
         open(writerObj);
    end
    %Setup the figure and start drawing
    figure();
    set(gcf,'color','w');
    set(gcf,'position',[0 0 1400 900]);
    grid on;
    axis equal;
    xlim([-1 1]);
    ylim([-1 1]);
    zlim([-1 1]);
    view(30,30);
    for i=1:20:size(js,2) %specify framerate
        gth=ComputeFK(js(1:end/2,i));
        DrawRobot(gth,0.04);
        title(t(i));
        drawnow;
        if (record==1)
            frame = getframe(gcf);
            writeVideo(writerObj,frame);
        end
    end
    if (record==1)
        close(writerObj);
    end
end