%Do a computed torque controller on the simulated manufacturing robot. Once
%without any linearization, once linearized with the actual parameters,
%once linearized with the identified parameters

%Need parameter_val to be filled.
kp=50;
kd=5;% random guess

InitParamsMB();
t=0:.01:10;%timestep is the same as always
%x=[1 0 0 0 1 0 0 0 1 0 0 0 1 0 0 0];
x=randn(1,24)*.3; %simple for now
[t,js_des]=ComputeJs(t,x);
tau_ct=zeros(DOF,1);  %_ct because my namespace is cluttered....
js_ct=zeros(2*DOF,1);
js_control=zeros(2*DOF,1);
js_nolin=zeros(2*DOF,1);

derive=1;
J=DeriveBodyJacobians(DOF,q,w,g,derive);
D=DeriveD(J, I,m, DOF,derive);
C=DeriveC(D,DOF,derive);
gth=DeriveFK(DOF,g,w,q,derive);
N=DeriveN(DOF,gth,gravity,fc,fv,m,derive);
eom=DeriveEOM(D,C,N,DOF,1);
eom3=DeriveEOM3(D,C,N,DOF,1);
ODEoptions=odeset('RelTol',1e-6);
for i=1:(size(t,2)-1)
     %PD+feedforward control
     aq=js_des(DOF*2+1:DOF*3,i)+kd*(js_des(DOF+1:DOF*2,i)-js_ct(DOF+1:DOF*2,end))+kp*(js_des(1:DOF,i)-js_ct(1:DOF,end));
     aq_control=js_des(DOF*2+1:DOF*3,i)+kd*(js_des(DOF+1:DOF*2,i)-js_control(DOF+1:DOF*2,end))+kp*(js_des(1:DOF,i)-js_control(1:DOF,end));
     %linearize
     tau_ct=[tau_ct, ComputeEOM3(aq,js_ct(1:DOF*2,end))];
     
     
     
     [~,js_temp]=ode45(@ComputeEOM,[t(i),t(i+1)],js_ct(:,end),ODEoptions,tau_ct(:,end));
     [~,js_temp2]=ode45(@(t,js) [js(1+DOF:2*DOF);aq_control],[t(i),t(i+1)],js_control(:,end));
     [~,js_temp3]=ode45(@ComputeEOM,[t(i),t(i+1)],js_nolin(:,end),ODEoptions,aq(:,end));
     js_ct=[js_ct js_temp(end,:)']; %only last element is recorded
     js_control=[js_control js_temp2(end,:)'];
     js_nolin=[js_nolin js_temp3(end,:)'];
end
 
%Identified parameters
InitBadParamsMB();
th=sym('th',[DOF,1]);
thd=sym('thd',[DOF,1]);
thdd=sym('thdd',[DOF,1]);
assume(th,'real');
assume(thd,'real');
assume(thdd,'real');
derive=0;
J=DeriveBodyJacobians(DOF,q,w,g,derive);
D=DeriveD(J, I,m, DOF,derive);
C=DeriveC(D,DOF,derive);
gth=DeriveFK(DOF,g,w,q,derive);
N=DeriveN(DOF,gth,gravity,fc,fv,m,derive); 
N=expand(N); %Done because subs el stupido
for i=size(allKeyTerms,2):-1:1 %backwards so we don't skip sub any m before m*r terms
    D=subs(D,str2sym(allKeyTerms{i}),parameter_val(i));
    C=subs(C,str2sym(allKeyTerms{i}),parameter_val(i));
    N=subs(N,str2sym(allKeyTerms{i}),parameter_val(i));

end
derive=1;
%eom=DeriveEOM(D,C,N,DOF,1); We don't want this line because we want the
%dynamics from the real parameters
eom3=DeriveEOM3(D,C,N,DOF,1);
tau_ct2=zeros(DOF,1);  %
js_ct2=zeros(2*DOF,1);
t=t(3:end-2);
for i=1:(size(t,2)-1)
     %PD+feedforward control
     aq=js_des(DOF*2+1:DOF*3,i)+kd*(js_des(DOF+1:DOF*2,i)-js_ct2(DOF+1:DOF*2,end))+kp*(js_des(1:DOF,i)-js_ct2(1:DOF,end));
     %linearize
     tau_ct2=[tau_ct2, ComputeEOM3(aq,js_ct2(1:DOF*2,end))];
     
     [t_temp,js_temp]=ode45(@ComputeEOM,[t(i),t(i+1)],js_ct2(:,end),ODEoptions,tau_ct2(:,end));
     js_ct2=[js_ct2 js_temp(end,:)']; %only last element is recorded
end

for i=1:4
    f(i)=figure();
    hold on;
    plot(t,js_ct(i,:));
    plot(t,js_des(i,:));
    plot(t,js_control(i,:));
    plot(t,js_ct2(i,:));
    plot(t,js_nolin(i,:));
    title(strcat('Joint ',string(i),' Computed Torque Controller Trajectory'));
    xlabel('Time (s)');
    ylabel('Position (rad)');
    L=legend('CT with Real Parameters', 'PD control', 'Command', 'CT with ID Parameters','No linearization');
    L.Location='northeastoutside';


end

