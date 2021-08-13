%This is for bags collected via command line, aka
%rosbag record -a

%Takes the bag file and initializes the variables t, js, and eff

%if filt=0, filter nothing
%if filt=1, filter both the joint state and the current
%if filt=2, filter only joint state
function [t,js,eff]=FullBagToMatlab(filename,DOF,filt)
    bag=rosbag(filename);
    
    pos=readMessages(select(bag,'Topic','/joint_pos_actual'));
    vel=readMessages(select(bag,'Topic','/joint_vel_actual'));
    cur=readMessages(select(bag,'Topic','/joint_effort_actual'));
    t=[];
    t_count=1;
    t_start=bag.StartTime; %the first time stamp
    %for simplicity, we assume pos, vel, and eff are collected at the same
    %time, so we only accumulate one time (t) vector
    for i=1:size(pos,1)-2 % truncated slightly because sometimes there is one less position collected than the other values
        js(1:DOF,i)=pos{i}.Data(1:DOF);
        eff(1:DOF,i)=cur{i}.Data(1:DOF);
        js(1+DOF:2*DOF,i)=vel{i}.Data(1:DOF);
    end
    for i=1:size(bag.MessageList,1)
        temp_cell=cellstr(bag.MessageList{i,2});
        
        if strcmp(temp_cell{1,1},'/joint_pos_actual')
            t(1,t_count)=bag.MessageList{i,1}-t_start;
            t_count=1+t_count;
        end
    end
    t=t(:,1:size(js,2));
    %differentiate for acceleration, not necessarily that accurate and
    %might need filtering
    js(1+2*DOF:3*DOF,1)=zeros(DOF,1);
    js(1+2*DOF:3*DOF,end)=zeros(DOF,1);
    for i=2:size(t,2)-1
        js(1+2*DOF:3*DOF,i)=(js(1+DOF:2*DOF,i+1)-js(1+DOF:2*DOF,i-1))/(t(i+1)-t(i-1));
    end
    
    %if filt option=1, joint state filtered with fourier filter and current
    %filtered with lowpass
    if filt==1
        [t,js]=ComputeFilteredJs(t,js, DOF);
        eff=eff(:,3:end-2); %to match size with t
        for i=1:DOF
            eff(i,:)=lowpass(eff(i,:),.1);
        end
    end
    if filt==2
        [t,js]=ComputeFilteredJs(t,js, DOF);
        eff=eff(:,3:end-2); %to match size with t        
    end
end
