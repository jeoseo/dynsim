%Executable for identifying just the 4th joint of the robot


%ipm=identified parameters map, keys gives the names and values gives the
%values
%rms_tau,rms_eff give the rms errors of each respectively
clear all;
close all;
addpath("GEN/");

InitUnknownParamsMBJoint4(); %I plug in what I "should" know about the robot, and represent the rest with symbolic
[t,js,eff]=BagToMatlab('rosbags_MB/DPI_2.bag',4,0); %Replaces SimMB, we have real values now
sim=false; %tells the rest of the program to not add noise into the signal for simulation purposes

%cut out data from first 3 joints as we are only concerned about the last
%one
js=[js(4,:);js(8,:);js(12,:)];
eff=eff(4,:);
%convert effort into torque
[tau,mode]=ComputeTauFromEff(eff,js,a);
%tau=eff*4.5*a(8);
IdentifyMB(); %I refactor the dynamics equation to solve for the inertial parameters with LMS and the js, tau vectors

th=sym('th',[DOF,1]);
thd=sym('thd',[DOF,1]);
thdd=sym('thdd',[DOF,1]);
assume(th,'real');
assume(thd,'real');
assume(thdd,'real');
%We need to substitute our new values in D, C, N
N=expand(N); %Done because subs el stupido
for i=size(allKeyTerms,2):-1:1 %backwards so we don't skip sub any m before m*r terms
    D=subs(D,str2sym(allKeyTerms{i}),parameter_val(i));
    C=subs(C,str2sym(allKeyTerms{i}),parameter_val(i));
    N=subs(N,str2sym(allKeyTerms{i}),parameter_val(i));
end


derive=1;
eom2=DeriveEOM2(D,C,N,DOF,derive);
sim_tau=[];
for i=1:(size(js,2))
    sim_tau=[sim_tau,double(ComputeEOM2(js(1:DOF,i),js(DOF+1:DOF*2,i),js(2*DOF+1:3*DOF,i)))];
end
sim_eff=ComputeEffFromTau(sim_tau,js,a);
f1=figure;
hold on;
plot(t,tau(1,:));
plot(t,sim_tau(1,:));
plot(t,eff(1,:));
plot(t,sim_eff(1,:));
title('')
xlabel('Time (s)')
ylabel('Torque (Nm)')
L=legend('Real Torque of Joint 4','Predicted Torque of Joint 4','Real Current of Joint 4','Predicted Current of Joint 4');
L.Location='northeastoutside';

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

rms_tau=rms(tau-sim_tau)
rms_eff=rms(eff-sim_eff)


