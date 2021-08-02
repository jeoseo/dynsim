%Main executable for simulation of Torque testing arm
clear all;
close all;
InitParamsTTA(); %I plug in all known robot parameters
[t,js,eff]=BagToMatlab('rosbags/sinusoids3.bag',1); % Get the joint trajectory and corresponding current

%[t,js,eff]=FullBagToMatlab('rosbags2/const_vel_levels.bag'); %two different types of bags were created, so second parser here



SimGeneric(); %I simulate the robot with some trajectory to find theoretical torque tau

f=figure;
hold on;
plot(t,js(1,:));
plot(t,js(2,:));
plot(t,js(3,:));
title('Current Torque Identification Arm')
xlabel('Time (s)')
ylabel('rad,rad/s,rad/s^2')
L=legend(["Position","Velocity","Acceleration"]);
L.Location='northeastoutside';

f1=figure;
hold on;


tau_pred=[];    
tau_pred(1)=0;
%motor 1
%a=[ 4.6024   10.9659    0.3651    0.8602    1.9442    0.0355    5.0877]; %made with sin2
a=[ 4.6798   10.9292    0.5303    0.7629    1.9454  0 4.3]; %made with sin3 

%motor 2
%a=[  4.5304    9.5698    0.3443    0.9000    2.0161    2.3257    5.9820];

mode=[];
mode(1)=1;
for i=2:size(tau,2)
    slope1=a(1);
    slope2=a(2);
    if abs(js(2,i))>.1 %outside stribeck effect, .1 rad/sec is where i have it for now
        f=sign(js(2,i))*a(4)+js(2,i)*a(5);
        if abs(f) > a(7) %saturation point
            f=a(7)*sign(f);
        end
    else %inside stribeck effect
        f=sign(js(2,i))*a(6)-js(2,i)*(a(6)-a(4)-.1*a(5))/.1;
    end
    tau_pred(i)=(eff(1,i))*slope2-slope2/slope1*(f+a(3)*js(3,i));   
    if (tau_pred(i)>=0 &&  js(2,i) >=0) || (tau_pred(i)<=0 &&  js(2,i) <=0 )
        tau_pred(i)=(eff(1,i))*slope1-f-a(3)*js(3,i);
    end
    %some fudging to reduce spikes
    fudge=1;
    if abs(tau_pred(i)-tau_pred(i-1)) > fudge && i> 5
        tau_pred(i)=tau_pred(i-1)+sign(tau_pred(i)-tau_pred(i-1))*fudge;
    end
end
eff_pred=[];
for i=1:size(tau,2)
    slope1=a(1);
    slope2=a(2);
    if abs(js(2,i))>.1 %outside stribeck effect, .1 rad/sec is where i have it for now
        f=sign(js(2,i))*a(4)+js(2,i)*a(5);
        if abs(f) > a(7) %saturation point
            f=a(7)*sign(f);
        end
    else %inside stribeck effect
        f=sign(js(2,i))*a(6)-js(2,i)*(a(6)-a(4)-.1*a(5))/.1;
    end
    mode(i)=2;
    eff_pred(i)=(tau(i)+slope2/slope1*(f+a(3)*js(3,i)))/slope2;
    if (tau(i)>=0 &&  js(2,i) >=0) || (tau(i)<=0 &&  js(2,i) <=0 )
        mode(i)=1;
        eff_pred(i)=(tau(i)+a(3)*js(3,i)+f)/slope1;
    end
end




plot(t,tau_pred);
plot(t,eff(1,:));
plot(t,tau(1,:));
plot(t,eff_pred);
title('Dynamixel Torque Current Identification')
xlabel('Time (s)')
ylabel('Nm,A')
L=legend(["Estimated Torque","Measured Current","Computed Torque","Estimated Current"]);
L.Location='northeastoutside';

eff1=[];
eff2=[];
eff3=[];
eff4=[];
vel1=[];
vel2=[];
vel3=[];
vel4=[];
tau1=[];
tau2=[];
tau3=[];
tau4=[];

for i=1:size(t,2)
    if js(2,i) > 0 && tau(i)>fc+fv*js(2,i)
        eff1=[eff1 eff(i)];
        vel1=[vel1 js(2,i)];
        tau1=[tau1 tau(i)];
    elseif js(2,i) < -0 && tau(i)>-fc+fv*js(2,i)
        eff2=[eff2 eff(i)];
        vel2=[vel2 js(2,i)];
        tau2=[tau2 tau(i)];
    elseif js(2,i) < -0 && tau(i)<-fc+fv*js(2,i)
        eff3=[eff3 eff(i)];
        vel3=[vel3 js(2,i)];
        tau3=[tau3 tau(i)];
    elseif js(2,i) > 0 && tau(i)<fc+fv*js(2,i)
        eff4=[eff4 eff(i)];
        vel4=[vel4 js(2,i)];
        tau4=[tau4 tau(i)];
    end
end
f2=figure;
title("Current vs Torque Scatter")
hold on;
scatter(eff1,tau1);
scatter(eff2,tau2);
scatter(eff3,tau3);
scatter(eff4,tau4);
L=legend(["V+,T+","V-,T+","V-,T-","V+,T-"]);
xlabel('Current (A)')
ylabel('Torque (Nm)')

%fitlm(table(eff1',tau1')) 
%fitlm(table(eff2',tau2'))
%fitlm(table(eff3',tau3')) 
%fitlm(table(eff4',tau4'))

rms_tau=rms(tau_pred-tau)
rms_eff=rms(eff_pred-eff)

