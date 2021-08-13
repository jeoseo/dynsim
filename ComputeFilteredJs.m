%Filters a noisy sinusoid signal
%t_out is outputted in case the length of the data is truncated (should be
%the same most of the time)
%filt_js is the filtered joint state

function [t_out,filt_js]=ComputeFilteredJs(t,js, DOF)   
    %assume the signal has constant time step
    dt=(t(end)-t(1))/size(t,2);
    for i=1:DOF
        fft1=fft(js(i,:));
        %Depending on the sample size and frequency, the discret fourier
        %transform forms different boxes. Because of this, we find the last
        %major peak and filter the rest out
        max_fft=max(abs(fft1));
        for j=20:-1:1
            if abs(fft1(j))>max_fft*.01
                break;
            end
        end
        fft1(j+3:end)=0;
        fft1(2:j+2)=fft1(2:j+2)*2; %We double the values because of MATLAB computes a double sided spectrum (See examples in https://www.mathworks.com/help/matlab/ref/fft.html)
        filt_js(i,:)=real(ifft(fft1));
    end
    for i=1:DOF %taking the derivatives
        filt_js(i+DOF,2:end-1)=(filt_js(i,3:end)-filt_js(i,1:end-2))/(2*dt);
        filt_js(i+2*DOF,2:end-1)=(filt_js(i+DOF,3:end)-filt_js(i+DOF,1:end-2))/(2*dt);
    end
    t_out=t;
    filt_js=filt_js(:,3:end-2);
    filt_js=[filt_js(:,1),filt_js(:,1),filt_js,filt_js(:,end),filt_js(:,end)]; %pad the beginning and end to make up for the derivative truncating values
end