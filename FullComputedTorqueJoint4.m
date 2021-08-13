%This script plots the trajectory taken for the computed torque control for
%just the 4th joint of the manufacturing robot
clear all;
close all;
[t,js,eff]=FullBagToMatlab('rosbags_MB/chirp_computed_torque.bag',4,0);

 
f=figure;
hold on;


move_start=0; %to mark where we started moving
%the desired position depends on what was originally commnanded; there is 
%no ros topic that holds this information so it is just kinda up to us to
%remember what the intended trajectory was
for i=1:size(js,2)
    if move_start==0
        if js(8,i)~= 0
            move_start=t(i);
        end
        js_des(i)=-pi/4;
    else
        js_des(i)=-pi/4*cos((t(i)-move_start)*(t(i)-move_start+5)/10);
    end
end
t=t(150:700);
js=js(:,150:700);
js_des=js_des(:,150:700);
plot(t-move_start,js(4,:));
plot(t-move_start,js_des(1,:));

title('Computed Torque for Joint 4')
xlabel('Time (s)')
ylabel('rad')
L=legend(["Position","Desired Position"]);
L.Location='northeastoutside';








