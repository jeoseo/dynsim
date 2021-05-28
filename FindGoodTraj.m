%requires phi, get this from FullMB
rng(12345);
addpath(genpath('GEN')); %Add path for generated functions

%GlobalSearch except this doesn't work well
% gs = GlobalSearch('MaxTime',1000);
 x0=zeros(24,1);
% options = optimoptions('fmincon','Display','iter');
% problem = createOptimProblem('fmincon','x0',x0,'objective',@ComputeDOptimal,'lb', -.5*ones(24,1),'ub',.5*ones(24,1),'nonlcon',@constr,'options',options);
% x = run(gs,problem);
%x= fmincon(@ComputeDetF, x0, [], [], [], [], -10*ones(1,16),10*ones(1,16),@constr,options);
options=optimoptions('surrogateopt','Display','iter','MaxTime',1000,'UseParallel',true,'MinSampleDistance',.1,'MinSurrogatePoints',24*10,'MaxFunctionEvaluations',24*1000);

x=surrogateopt(@combine,-1*ones(24,1),1.1*ones(24,1),options);

function f=combine(x)
    f.Fval=ComputeDOptimal(x);
    f.Ineq=constr(x);
end

function minimize=ComputeDOptimal(x)
    Tf=10; %end of sim (start of sim should always be t=0)
    dt=.01; %time step for recording joint state
    t=0:dt:Tf; %timesteps for changing torque
    [t_out,js]=ComputeJs(t,x);
    DOF=4;
    F=[];
    for i=1:10:size(js,2)
        F=[F;ComputePhiMB(js(1:4,i),js(5:8,i),js(9:12,i))]; %We stack our phi's together to do least mean squares
    end
    [~,indcolF]=rref(F);
    if size(indcolF,2)==33
        Fshrunk=F(:,indcolF);
        %minimize=max(diag(inv(Fshrunk'*Fshrunk)));
        minimize=-log(det(Fshrunk'*Fshrunk));
        %[U,S,V]=svd(Fshrunk'*Fshrunk);
        %minimize=log(max(diag(S));
    else
        minimize=1000;
    end
    
end

function c = constr(x)
    Tf=10; %end of sim (start of sim should always be t=0)
    dt=.01; %time step for recording joint state
    t=0:dt:Tf; %timesteps for changing torque
    [t_out,js]=ComputeJs(t,x);
   % c <= 0
   % ceq = 0 
   %joint upper and lower bounds
   c(1) = max(js(1,:))-pi;
   c(2) = -pi-min(js(1,:));
   c(3) = max(js(2,:))-pi/2;
   c(4) = 0-min(js(2,:));
   c(5) = max(js(3,:));
   c(6) = -pi/2-min(js(3,:));
   c(7) = max(js(4,:))-pi/2;
   c(8) = -pi/2-min(js(4,:));
   
   %abs velocity upper bound;
   
   c(9)=max(abs(js(5,:)))-3.04;
   c(10)=max(abs(js(6,:)))-.73;
   c(11)=max(abs(js(7,:)))-1.22;
   c(12)=max(abs(js(8,:)))-1.22;
   
   %abs torque bounds
   
   tau_lim=max(abs(ComputeEOM2(js(1:4,:),js(5:8,:),js(9:12,:))),[],2);
   c(13)=tau_lim(1)-40; 
   c(14)=tau_lim(2)-4.167*40;
   c(15)=tau_lim(3)-2.5*40;
   c(16)=tau_lim(4)-2.5*40;
   
end

