%Takes the bag file and initializes the variables t, js, and eff
%The bag file should have the topics pos,vel,and eff.
%This bag file is generated by the script id_motor.py and dynamic_id.py


%if filt=0, filter nothing
%if filt=1, filter both the joint state and the current
%if filt=2, filter only joint state

function [t,js,eff]=BagToMatlab(filename,DOF,filt)
    bag=rosbag(filename);
    
    pos=readMessages(select(bag,'Topic','/pos'));
    vel=readMessages(select(bag,'Topic','/vel'));
    effort=readMessages(select(bag,'Topic','/eff'));
    t=[];
    t_start=bag.MessageList{1,1}; %the first time stamp
    for i=1:single((size(bag.MessageList,1)/3))
        t=[t, bag.MessageList{1+3*(i-1),1}-t_start];
        js(1:DOF,i)=pos{i}.Data(1:DOF);
        eff(1:DOF,i)=effort{i}.Data(1:DOF);
        js(1+DOF:2*DOF,i)=vel{i}.Data(1:DOF); %currently only using
        %position because of the FFT filtering
    end
    %differentiate for acceleration, not necessarily that accurate
    js(1+2*DOF:3*DOF,1)=zeros(DOF,1);
    js(1+2*DOF:3*DOF,end)=zeros(DOF,1);
    for i=2:size(t,2)-1
        js(1+2*DOF:3*DOF,i)=(js(1+DOF:2*DOF,i+1)-js(1+DOF:2*DOF,i-1))/(t(i+1)-t(i-1));
    end

    if filt==1
        [t,js]=ComputeFilteredJs(t,js, DOF);
        for i=1:DOF
            eff(i,:)=lowpass(eff(i,:),.1);
        end
    end
    if filt==2
        [t,js]=ComputeFilteredJs(t,js, DOF);  
    end
end
