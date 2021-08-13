%Converts the output torque to the input current

%js is the joint state of the output shaft, hence after the gear ratio has
%been applied

%a is a motor parameter vector; see the current torque identification
%section in my report for details 
%a(1),a(2): torque constants
%a(3): rotor inertia
%a(4): coulomb friction
%a(5): viscous friction
%a(6): stribeck friction
%a(7): saturation friction
%a(8): the gear ratio

%eff_pred is the predicted current from the motor (before the gear ratio)
%tau is the torque from the output shaft, so after the gear ratio
function eff_pred = ComputeEffFromTau(tau, js, a)
    eff_pred=[];
    js=js*a(8); %convert to joint state of motor shaft
    tau=tau/a(8);
    for i=1:size(tau,2)
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
        eff_pred(i)=(tau(i)+slope2/slope1*(f+a(3)*js(3,i)))/slope2;
        if (tau(i)>=0 &&  js(2,i) >=0) || (tau(i)<=0 &&  js(2,i) <=0 )
            eff_pred(i)=(tau(i)+a(3)*js(3,i)+f)/slope1;
        end
    end
    
end