%requires phi, get this from FullMB
rng default % For reproducibility
gs = GlobalSearch('NumTrialPoints',1000);
x0=[    0.0018    0.0276   -0.1285   -0.0151    0.9794   -0.1463    0.1335   -0.4630   -0.7548   -0.7416   -0.1347   -0.2028    0.1428   -1.0000   -0.7437    0.0067];

options = optimoptions('fmincon','Display','iter','Algorithm','active-set','MaxIterations',500, 'StepTolerance', 1e-10);
limit=[.25,.25,.25,.25,.5,.5,.5,.5,.5,.5,.5,.5,.5,.5,.5,.5];
problem = createOptimProblem('fmincon','x0',x0,'objective',@ComputeDOptimal,'lb', -10*ones(1,16),'ub',10*ones(1,16),'nonlcon',@constr,'options',options);
x = run(gs,problem);
%x= fmincon(@ComputeDetF, x0, [], [], [], [], -10*ones(1,16),10*ones(1,16),@constr,options);

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
    %hacky way to get rank to be full
    indcolF=[ 1     2     3     4     5     6     7     8     9    10    11    12    13    14    15    16    20    21    22    23    24    25    26    27  29   30     31    32    33    34    35    36    37];

    Fshrunk=F(:,indcolF);
    cov=(Fshrunk'*Fshrunk);
    [U,S,V]=svd(cov);
    minimize=max(diag(S));
end

function [c, ceq] = constr(x)
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
   ceq = [];
end