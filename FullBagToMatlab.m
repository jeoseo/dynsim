%This is for bags collect via command line, aka
%rosbag record -a
function [t,js,tau]=FullBagToMatlab(filename)
    bag=rosbag(filename);
    
    pos=readMessages(select(bag,'Topic','/joint_pos_actual'));
    vel=readMessages(select(bag,'Topic','/joint_vel_actual'));
    eff=readMessages(select(bag,'Topic','/joint_effort_actual'));
    DOF=1;
    t=[];
    t_count=1;
    t_start=bag.StartTime; %the first time stamp
    %for simplicity, we assume pos, vel, and eff are collected at the same
    %time, so we only accumulate one time (t) vector
    for i=1:size(pos,1)-2 % truncated slightly because sometimes there is one less position collected than the other values
        js(1,i)=pos{i}.Data(1);
        tau(1,i)=eff{i}.Data(1);
        js(2,i)=vel{i}.Data(1);
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
    
    %torque tends to be too noisy without this
    tau=lowpass(tau,.5);
    
    %[t,js]=ComputeFilteredJs(t,js, DOF);
    %tau=tau(3:end-2); %to match size with t
end
