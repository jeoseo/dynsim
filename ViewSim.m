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
    xlim([-1.2 1.2]);
    ylim([-1.2 1.2]);
    zlim([0 1.2]);
    view(30,30);
    for i=1:5:size(js,2) 
        gth=ComputeFK(js(1:end/2,i));
        DrawRobot(gth,0.08);
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