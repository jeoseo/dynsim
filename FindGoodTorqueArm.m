%This is the optimization routine for finding a good set of motor
%parameters to convert between current and torque

rng(232); %I changed this occaasionally just to see what other results were possible
addpath(genpath('GEN')); %Add path for generated functions
%Nothing should be attached to the motor while testing
[t,js,eff]=BagToMatlab('rosbags_detached/motor_4_id.bag',4,0); % Get the joint trajectory and corresponding current

%the joint state is shortened, because the bag file collects data for all 4
%joints, but we were only interested in the 4th joint
%This means that if you collect data for a different motor on the robot,
%THIS MUST BE CHANGED
joint_number=4;
DOF=4;
js=[js(joint_number,:);js(joint_number+DOF,:);js(joint_number+2*DOF,:)];
eff=eff(joint_number,:);

tau=zeros(1,size(t,2)); %no external torque, so all zeros

save 'FindGoodTorqueArmTraj.mat' t js eff tau

options=optimoptions('surrogateopt','CheckpointFile','checkpoint_current_torque.mat','Display','iter','UseParallel',true,'MinSampleDistance',.1,'MinSurrogatePoints',24*10,'MaxFunctionEvaluations',10000000000000000);
%To stop the optimization routine, press stop on the gui that pops up. The
%result will be located in a_work (the rest of these outputs are for
%debugging, see https://www.mathworks.com/help/gads/surrogateopt.html)
[a_work,fval,exitflag,output]=surrogateopt(@combine,[4.5-3,4.5-3,0.3, .5, 1, 0,2,2.5],[4.5+3,4.5+3,1,1.5,3,3,8,2.5],options);

%In case one wants to continue an optimization from where it was left off
%[a_work,fval,exitflag,output]=surrogateopt('checkpoint_torque_arm.mat',options );
function f=combine(x)
    f.Fval=ComputeRMSTau(x);
end

function minimize=ComputeRMSTau(a)
    load FindGoodTorqueArmTraj.mat t js eff tau;
    eff_pred=ComputeEffFromTau(tau,js,a);
    minimize=rms(eff_pred-eff);
end