 %Generates a sinusoid joint trajectory from a 16 parameter vector
%Since the data is discretized, derivates are done symmetrically and crop
%parts of the left and right hand side
function [t_out,js]=ComputeJs(t,x)
    dt=t(2)-t(1); %find time step
    js=[];
    for i=1:4
        js=[js; x(6*i-5)+x(6*i-4)*sin(.2*pi*t)+x(6*i-3)*sin(.4*pi*t)+x(6*i-2)*sin(.6*pi*t)+x(6*i-1)*sin(.8*pi*t)+x(6*i)*sin(pi*t)];
    end
    js=[js(:,2:end-1);(js(:,3:end)-js(:,1:end-2))/(2*dt)];
    js=[js(:,2:end-1);(js(5:8,3:end)-js(5:8,1:end-2))/(2*dt)];
    t_out=t(3:end-2);
end