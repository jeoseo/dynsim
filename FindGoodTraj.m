%Optimizes the excitation trajectory for the manufacturing base robot

%requires the GEN folder to be populated with ComputePhiMB
%this can be done by running FullSimMB
rng(2);
addpath(genpath('GEN')); %Add path for generated functions

%lower bound and upper bound of -.5 and .6 can be experimented with
options=optimoptions('surrogateopt','CheckpointFile','checkpoint2.mat','Display','iter','UseParallel',true,'MinSampleDistance',.1,'MinSurrogatePoints',24*10,'MaxFunctionEvaluations',24*1000);
[x_work,fval,exitflag,output]=surrogateopt(@combine,-.5*ones(24,1),.6*ones(24,1),options);
%[x_work,fval,exitflag,output]=surrogateopt('checkpoint.mat',options);
function f=combine(x)
    f.Fval=ComputeDOptimal(x);
    f.Ineq=constr(x);
end

function minimize=ComputeDOptimal(x)
    Tf=10; %end of sim (start of sim should always be t=0)
    dt=.01; %time step for recording joint state
    t=0:dt:Tf; %timesteps for changing torque
    [~,js]=ComputeJs(t,x);
    DOF=4;
    %Generate and reduce F to its independent columns
    F=[];
    for i=1:10:size(js,2)
        F=[F;ComputePhiMB(js(1:4,i),js(5:8,i),js(9:12,i))]; %We stack our phi's together to do least mean squares
    end
    [~,indcolF]=rref(F);
    if size(indcolF,2)==32 %hardcoded to make sure that we correctly find the right number of independent parameters
        Fshrunk=F(:,indcolF);
        minimize=real(-log(det(Fshrunk'*Fshrunk)));
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
   c(13)=tau_lim(1)-30; 
   c(14)=tau_lim(2)-4.167*30;
   c(15)=tau_lim(3)-2.5*30;
   c(16)=tau_lim(4)-2.5*30;
   
end

