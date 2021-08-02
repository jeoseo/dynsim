%Main executable for simulation of Torque testing arm
clear all;
close all;
InitParamsTTA(); %I plug in all known robot parameters=
[t,js,eff]=FullBagToMatlab('rosbags/chirp_computed_torque.bag');

 

f=figure;
hold on;

%plot(t,js(2,:));
%plot(t,js(3,:));
move_start=0; %to mark where we started moving

%the desired position depends on what was originally commnanded; there is 
%no ros topic that holds this information so it is just kinda up to us to
%remember what the intended trajectory was
% for i=1:size(js,2)
%    if move_start==0
%        if js(2,i) ~= 0
%             move_start=t(i);
%        end
%        js_des(i)=-pi/2;
%    else
%        js_des(i)=pi/2*sin(1+t(i)-move_start)-pi/2;
%    end
% end
for i=1:size(js,2)
    if move_start==0
        if js(2,i)~= 0
            move_start=t(i);
        end
        js_des(i)=0;
    else
        js_des(i)=-pi/4*cos((t(i)-move_start)*(t(i)-move_start+5)/10)+pi/4;
    end
end
plot(t(100:1025)-move_start,js(1,100:1025));
plot(t(100:1025)-move_start,js_des(100,100:1025));
    
title('Computed Torque for Hammer')
xlabel('Time (s)')
ylabel('rad')
L=legend(["Position","Desired Position"]);
L.Location='northeastoutside';








