%x=[x1;x2;x3], a 3x1 vector
%returns skew symmetric matrix

function hat=ComputeHat3(x)
    hat=[0 -x(3) x(2); x(3) 0 -x(1);-x(2) x(1) 0];
end