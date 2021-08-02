 %Generates a sinusoid joint trajectory from a 16 parameter vector
%Since the data is discretized, derivates are done symmetrically and crop
%parts of the left and right hand side
function [t_out,js]=ComputeJs(t,x)
    DOF=size(x,2)/6;
    dt=t(2)-t(1); %find time step
    js=[];
    for i=1:DOF
        js=[js; x(6*i-5)+x(6*i-4)*cos(.2*pi*t)+x(6*i-3)*cos(.4*pi*t)+x(6*i-2)*cos(.6*pi*t)+x(6*i-1)*cos(.8*pi*t)+x(6*i)*cos(pi*t)];
    end
    js=[js(:,2:end-1);(js(:,3:end)-js(:,1:end-2))/(2*dt)];
    js=[js(:,2:end-1);(js(DOF+1:2*DOF,3:end)-js(DOF+1:2*DOF,1:end-2))/(2*dt)];
    t_out=t(3:end-2);
end