%requires phi, get this from FullMB
rng(25);
addpath(genpath('GEN')); %Add path for generated functions
addpath(genpath('rosbags'));
InitParamsTTA(); %I plug in all known robot parameters
[t,js,eff]=BagToMatlab('rosbags2/sinusoids3.bag',1); % Get the joint trajectory and corresponding current
%[t,js,eff]=FullBagToMatlab('rosbags2/const_vel_levels.bag'); %two different types of bags were created, so second parser here

SimGeneric();
save 'FindGoodTorqueArmTraj.mat' t js eff tau

options=optimoptions('surrogateopt','CheckpointFile','checkpoint_torque_arm.mat','Display','iter','UseParallel',true,'MinSampleDistance',.1,'MinSurrogatePoints',24*10,'MaxFunctionEvaluations',10000000000000000);
[x_work,fval,exitflag,output]=surrogateopt(@combine,[3.5,8,.3,.6,0,0,3],[5,12,.6,.9,3,3,6],options);
%[x_work,fval,exitflag,output]=surrogateopt('checkpoint_torque_arm.mat',options );
function f=combine(x)
    f.Fval=ComputeRMSTau(x);
end


function minimize=ComputeRMSTau(a)
    %x contains torque coefficients,Irr,fc, fv, fs
    %constant 
    load FindGoodTorqueArmTraj.mat t js eff tau;
    tau_pred=[];
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
        if abs(tau_pred(i)-tau_pred(i-1)) > fudge&&  i>5
            tau_pred(i)=tau_pred(i-1)+sign(tau_pred(i)-tau_pred(i-1))*fudge;
        end
    end
    minimize=rms(tau_pred-tau);
end