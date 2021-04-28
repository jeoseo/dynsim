%Here, we assume that we have our map of inertial parameters (imp)
%and find the difference between our predicted and simulated torque on the
%original trajectory as well as one other onw
InitBadParamsMB();
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
diff_tau=new_tau-tau;
LMS=sqrt(sum(diff_tau(:).^2)/size(diff_tau(:),1));
title('Torque Comparison for Excitation Trajectory')
xlabel('Time (s)')
ylabel('Torque (Nm)')
legend('Real Torque of Joint 1','Predicted Torque of Joint 1')


%The 2nd trajectory, theoretically should be a lot worse
%For now, just to a random trajectory (may not be possible, but should
%still be accurate)
rng default;
x=rand(1,16)-0.5;
t=0:dt:Tf;
js2=[x(1)+x(2)*sin(.2*pi*t)+x(3)*sin(.4*pi*t)+x(4)*sin(.6*pi*t);
        x(5)+x(6)*sin(.2*pi*t)+x(7)*sin(.4*pi*t)+x(8)*sin(.6*pi*t);
        x(9)+x(10)*sin(.2*pi*t)+x(11)*sin(.4*pi*t)+x(12)*sin(.6*pi*t);
        x(13)+x(14)*sin(.2*pi*t)+x(15)*sin(.4*pi*t)+x(16)*sin(.6*pi*t)];
js2=[js2(:,2:end-1);(js2(:,3:end)-js2(:,1:end-2))/(2*dt)];
js2=[js2(:,2:end-1);(js2(5:8,3:end)-js2(5:8,1:end-2))/(2*dt)];

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
diff_tau2=new_tau2-tau2;
LMS2=sqrt(sum(diff_tau2(:).^2)/size(diff_tau2(:),1));
title('Torque Comparison for Random Trajectory')
xlabel('Time (s)')
ylabel('Torque (Nm)')
legend('Real Torque of Joint 1','Predicted Torque of Joint 1')

