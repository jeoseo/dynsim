%Generates a sinusoid joint trajectory from a 16 parameter vector
%Since the data is discretized, derivates are done symmetrically and crop
%parts of the left and right hand side
function [t_out,js]=ComputeJs(t,x)
    dt=t(2)-t(1); %find time step
    js=[x(1)+x(2)*sin(.2*pi*t)+x(3)*sin(.4*pi*t)+x(4)*sin(.6*pi*t);
        x(5)+x(6)*sin(.2*pi*t)+x(7)*sin(.4*pi*t)+x(8)*sin(.6*pi*t);
        x(9)+x(10)*sin(.2*pi*t)+x(11)*sin(.4*pi*t)+x(12)*sin(.6*pi*t);
        x(13)+x(14)*sin(.2*pi*t)+x(15)*sin(.4*pi*t)+x(16)*sin(.6*pi*t)];
    js=[js(:,2:end-1);(js(:,3:end)-js(:,1:end-2))/(2*dt)];
    js=[js(:,2:end-1);(js(5:8,3:end)-js(5:8,1:end-2))/(2*dt)];
    t_out=t(3:end-2);
end