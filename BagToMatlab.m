%Takes the bag file and initializes the variables t, js, and eff
%The bag file should have the topics pos,vel,and eff.

%For now, we always cut out a 10 second snippet, sampled at 100 Hz 
function [t,js,eff]=BagToMatlab(filename,DOF)
    bag=rosbag(filename);
    
    pos=readMessages(select(bag,'Topic','/pos'));
    vel=readMessages(select(bag,'Topic','/vel'));
    effort=readMessages(select(bag,'Topic','/eff'));
    %Assuming the trajectory went for 30 seconds, we take the middle 10
    %seconds
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
    eff=lowpass(eff,.5);
%     for i=1:DOF
%         js(i+2*DOF,:)=lowpass(js(i+2*DOF,:),.01);
%     end
    %[t,js]=ComputeFilteredJs(t,js, DOF);
    %eff=eff(3:end-2); %to match size with t
end