%Computes the 6x6 adjoint transform for some 4x4 homongenous transform
%[R phat*R; 0 R] for some T=[R p;0 1], where R is the rotation matrix, p is
%the translation vector

function adj=ComputeAdjoint(T)
    R=T(1:3,1:3);
    p=T(1:3,4);
    adj=[R ComputeHat3(p)*R;zeros(3) R];
end
