%Computes the 4x4 transformation matrix given a joint value and twist
%w is the joint twist (6x1, linear then angular)
%q is the joint value (scalar)
function exp=ComputeExpn(w,q)
    w1=w(1:3); %linear
    w2=w(4:6); %rotation
    w_hat=ComputeHat3(w2);
    %Rodriegez formula
    exp3=eye(3)+w_hat*sin(q)+w_hat^2*(1-cos(q));
    
    t=(eye(3)-exp3)*(cross(w2,w1))+w2*w2'*w1*q;
    exp=[exp3 t;0 0 0 1];
end
