%Main executable to generate figures for the identification of the current
%torque relationship
clear all;
close all;
InitSimParamsTTA(); %I plug in all known robot parameters
[t,js,eff]=BagToMatlab('rosbags_detached/motor_4_id.bag',4,0); % Get the joint trajectory and corresponding current
js=[js(4,:);js(8,:);js(12,:)];
eff=eff(4,:);

%The TTA (torque testing arm) ended up not being useful, so we just
%substitute zeros for the torque now
tau=zeros(1,size(t,2));
%SimGeneric(); %I simulate the robot with some trajectory to find theoretical torque tau

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


%insert other motor parameters here...
%motor 4 parameters
a=[4.6555    6.2920    0.5452    0.8938    2.0300    .2149   5.6903    2.5000];

[tau_pred,mode]=ComputeTauFromEff(eff,js(1:3,:),a);
eff_pred=ComputeEffFromTau(tau,js(1:3,:),a);

f1=figure;
hold on;
plot(t,tau_pred);
plot(t,eff(1,:));
plot(t,tau(1,:));
plot(t,eff_pred);
%plot(t,mode);
title('Dynamixel Torque Current Identification')
xlabel('Time (s)')
ylabel('Nm,A')
L=legend(["Torque from Measured Current","Measured Current","Simulated Torque","Current from Simulated Torque"]);
L.Location='northeastoutside';


rms_tau=rms(tau_pred-tau)
rms_eff=rms(eff_pred-eff)

