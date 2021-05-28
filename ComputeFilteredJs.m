%Applies noise to the position of the joint state, then applies a filter
%and takes derivatives to more accurately represent the kind of data we
%might get.
%Low pass filters make the boundary wonky, so n is the number of numbers to
%clip on either side.
%  function [t_out,filt_js]=ComputeFilteredJs(t,js, DOF,sigma,n)
%      dt=t(2)-t(1); %find dt
%      t_out=t;
%      for i=1:DOF
%          noise_js=sigma.*randn(1,size(js,2))+js(i,:); %add gaussian noise
%          temp=lowpass(noise_js,.3,1/dt); %frequencies should be .1, .2, and .3
%          filt_js(i,:)=temp(1+n:end-n);
%      end
%      t_out=t_out(1+n:end-n);
%      for i=1:DOF
%          temp=lowpass((filt_js(i,5:end)+filt_js(i,4:end-1)*8-filt_js(i,2:end-3)*8-filt_js(i,1:end-4))/(12*dt),.3,1/dt);
%          filt_js(i+DOF,2+n+1:end-n-2)=temp(1+n:end-n);
%      end
%      filt_js=filt_js(:,1+n+2:end-n-2);
%      t_out=t_out(1+2+n:end-n-2);
%      for i=1:DOF
%          temp=lowpass((filt_js(i+DOF,3:end)-filt_js(i+DOF,1:end-2))/(2*dt),.3,1/dt);
%          filt_js(i+2*DOF,1+n+1:end-1-n)=temp(1+n:end-n);
%      end
%      filt_js=filt_js(:,1+n+1:end-n-1);
%      t_out=t_out(1+1+n:end-n-1);
%  end

function [t_out,filt_js]=ComputeFilteredJs(t,js, DOF,sigma)
   %showing that we can rederive the input function even with noise...
   %not sure if this is usable in real life as the noise not just going to
   %be white/gaussian
    dt=t(2)-t(1);
    for i=1:DOF
        noise_js=sigma.*randn(1,size(js,2))+js(i,:); %add the noise
        fft1=fft(noise_js);
%         fft1(3)=0;
%         fft1(5)=0;
%         fft1(7:end)=0;%remove any frequencies that shouldn't exist
%         fft1(2)=fft1(2)*2; %*2 b/c of the double sided spectrum (see https://www.mathworks.com/help/matlab/ref/fft.html)
%         fft1(4)=fft1(4)*2;
%         fft1(6)=fft1(6)*2;
        fft1(7:end)=0;
        fft1(2:6)=fft1(2:6)*2;
        filt_js(i,:)=real(ifft(fft1));
    end
    for i=1:DOF %taking the derivatives
        filt_js(i+DOF,2:end-1)=(filt_js(i,3:end)-filt_js(i,1:end-2))/(2*dt);
        filt_js(i+2*DOF,2:end-1)=(filt_js(i+DOF,3:end)-filt_js(i+DOF,1:end-2))/(2*dt);
    end
    t_out=t(3:end-2);
    filt_js=filt_js(:,3:end-2);
end