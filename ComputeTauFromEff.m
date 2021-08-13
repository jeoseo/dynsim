%Computes the torque from the joint effort and state from a single joint

%takes a parameter vector "a" to define the current torque relationship
%a is a motor parameter vector; see the current torque identification
%section in my report for details 
%a(1),a(2): torque constants
%a(3): rotor inertia
%a(4): coulomb friction
%a(5): viscous friction
%a(6): stribeck friction
%a(7): saturation friction
%a(8): the gear ratio


%js is the joint state of the output shaft, hence after the gear ratio has
%been applied
%eff is the current measured from the motor (before the gear ratio)
%tau_pred is the predicted torque from the output shaft, so after the gear
%ratio
%mode is an optional debugging variable to show when the motor is being
%backdriven
function [tau_pred,mode] = ComputeTauFromEff(eff,js,a)
    tau_pred=[];
    mode=[];
    js=js*a(8); %account for gear ratio
    tau_pred(1)=0;
    mode(1)=0;
    %iterate over the current, skipping the first one because of what
    %happens on line 52
    for i=2:size(eff,2) 
        slope1=a(1);
        slope2=a(2);
        if abs(js(2,i))>.1 %outside stribeck effect, .1 rad/sec is where i have it for now
            f=sign(js(2,i))*a(4)+js(2,i)*a(5);
            if abs(f) > a(7) %saturation pfoint
                f=a(7)*sign(f);
            end
        else %inside stribeck effect
            f=sign(js(2,i))*a(6)-js(2,i)*(a(6)-a(4)-.1*a(5))/.1;
        end
        mode(i)=2;
        tau_pred(i)=(eff(1,i))*slope2-slope2/slope1*(f+a(3)*js(3,i));   
        if (tau_pred(i)>=0 &&  js(2,i) >=0) || (tau_pred(i)<=0 &&  js(2,i) <=0 )
            mode(i)=1;
            tau_pred(i)=(eff(1,i))*slope1-(f+a(3)*js(3,i));
        end
        
        %some fudging to reduce spikes
        %limits the maximum change in torque between timesteps to "fudge",
        %except at the beginning of the data
        fudge=1;
        if abs(tau_pred(i)-tau_pred(i-1)) > fudge && i> 5
            tau_pred(i)=tau_pred(i-1)+sign(tau_pred(i)-tau_pred(i-1))*fudge;
        end
    end
    tau_pred=tau_pred*a(8);
end