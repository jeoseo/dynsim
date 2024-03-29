%Executable for identifying the full, real robot
%this has never actually been run, so expect errors


%ipm=identified parameters map, keys gives the names and values gives the
%values
%rms_tau,rms_eff give the rms errors of each respectively
clear all;
close all;
addpath("GEN/");

InitUnknownParamsMB(); %I plug in what I "should" know about the robot, and represent the rest with symbolic
[t,js,eff]=BagToMatlab('rosbags_MB/????.bag',4,2); %Replace with actually collected bag
sim=false; %tells the rest of the program to not add noise into the signal for simulation purposes


%convert effort into torque
for i=1:DOF
    [tau(i,:),mode(i,:)]=ComputeTauFromEff(eff(i,:),[js(i,:);js(i+DOF,:);js(i+2*DOF,:)],a(i,:)); 
end
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
for i=1:DOF
    [eff(i,:),mode(i,:)]=ComputeEffFromTau(tau(i,:),[js(i,:);js(i+DOF,:);js(i+2*DOF,:)],a(i,:)); 
end
f1=figure;
hold on;
plot(t,tau(1,:));
plot(t,sim_tau(1,:));
plot(t,eff(1,:));
plot(t,sim_eff(1,:));
title('')
xlabel('Time (s)')
ylabel('Torque (Nm)')
L=legend('Real Torque of Joint 1','Predicted Torque of Joint 1','Real Current of Joint 1','Predicted Current of Joint 1');
L.Location='northeastoutside';

rms_tau=rms(tau-sim_tau)
rms_eff=rms(eff-sim_eff)


