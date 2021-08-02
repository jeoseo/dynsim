%Here, we assume that we have our map of inertial parameters (imp)
%and find the difference between our predicted and simulated torque on the
%original trajectory as well as one other onw
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
new_tau=[];
for i=1:(size(js,2))
    new_tau=[new_tau,double(ComputeEOM2(js(1:DOF,i),js(DOF+1:DOF*2,i),js(2*DOF+1:3*DOF,i)))];
end
f1=figure;
hold on;
plot(t(3:end-2),tau(1,:));
plot(t(3:end-2),new_tau(1,:));
plot(t(3:end-2),tau(2,:));
plot(t(3:end-2),new_tau(2,:));
plot(t(3:end-2),tau(3,:));
plot(t(3:end-2),new_tau(3,:));
plot(t(3:end-2),tau(4,:));
plot(t(3:end-2),new_tau(4,:));
diff_tau=new_tau-tau;
LMS=sqrt(sum(diff_tau(:).^2)/size(diff_tau(:),1));
title('Torque Comparison for Excitation Trajectory')
xlabel('Time (s)')
ylabel('Torque (Nm)')
L=legend('Real Torque of Joint 1','Predicted Torque of Joint 1',...
    'Real Torque of Joint 2','Predicted Torque of Joint 2',...
    'Real Torque of Joint 3','Predicted Torque of Joint 3',...
    'Real Torque of Joint 4','Predicted Torque of Joint 4');
L.Location='northeastoutside';


%The 2nd trajectory, theoretically should be a lot worse
%For now, just to a random trajectory (may not be possible, but should
%still be accurate)
%This one actually follows the joint constraints
x=[-0.3070   -0.1687    0.0518    0.0637   -0.0832    0.0193    0.5276   -0.3280    0.0611   -0.1366    0.0376    0.1634   -0.5388 0.0528   -0.0911   -0.1833   -0.0892    0.2621    0.2700   -0.1716    0.0135   -0.0529    0.4046    0.0466];
t=0:dt:Tf;
[t_out,js2]=ComputeJs(t,x);

%Compute predicted torque for new trajectory
new_tau2=[];
for i=1:(size(js,2))
    new_tau2=[new_tau2,double(ComputeEOM2(js2(1:DOF,i),js2(DOF+1:DOF*2,i),js2(2*DOF+1:3*DOF,i)))];
end

%Now compute "real" torque for new trajectory
InitParamsMB();
derive=1;
J=DeriveBodyJacobians(DOF,q,w,g,derive);
D=DeriveD(J, I,m, DOF,derive);
C=DeriveC(D,DOF,derive);
gth=DeriveFK(DOF,g,w,q,derive);
N=DeriveN(DOF,gth,gravity,fc,fv,m,derive);
eom2=DeriveEOM2(D,C,N,DOF,derive);
tau2=[];
for i=1:(size(js,2))
    tau2=[tau2,double(ComputeEOM2(js2(1:DOF,i),js2(DOF+1:DOF*2,i),js2(2*DOF+1:3*DOF,i)))];
end

f2=figure;
hold on;
plot(t,tau2(1,:));
plot(t,new_tau2(1,:));
plot(t,tau2(2,:));
plot(t,new_tau2(2,:));
plot(t,tau2(3,:));
plot(t,new_tau2(3,:));
plot(t,tau2(4,:));
plot(t,new_tau2(4,:));
diff_tau2=new_tau2-tau2;
LMS2=sqrt(sum(diff_tau2(:).^2)/size(diff_tau2(:),1));
title('Torque Comparison for Random Trajectory')
xlabel('Time (s)')
ylabel('Torque (Nm)')
L=legend('Real Torque of Joint 1','Predicted Torque of Joint 1',...
    'Real Torque of Joint 2','Predicted Torque of Joint 2',...
    'Real Torque of Joint 3','Predicted Torque of Joint 3',...
    'Real Torque of Joint 4','Predicted Torque of Joint 4');
L.Location='northeastoutside';


