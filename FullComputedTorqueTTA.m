%This script plots the computed torque control of the 4th motor when it was
%attached to the hammer arm
clear all;
close all;
[t,js,eff]=FullBagToMatlab('rosbags_4/chirp_computed_torque.bag',1,0);


f=figure;
hold on;

move_start=0; %to mark where we started moving
%the desired position depends on what was originally commnanded; there is 
%no ros topic that holds this information so it is just kinda up to us to
%remember what the intended trajectory was
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

%limit the window to where the trajectory was happening
t=t(100:1020);
js=js(:,100:1020);
js_des=js_des(:,100:1020);
plot(t-move_start,js(1,:));
plot(t-move_start,js_des(1,:));

title('Computed Torque for Hammer')
xlabel('Time (s)')
ylabel('rad')
L=legend(["Position","Desired Position"]);
L.Location='northeastoutside';








