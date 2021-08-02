%------------,-------------------------------------------------------------
%                              main.m
%          UNITS: kg, m, sec
%          Script to calculate wrench estimation errors for a given set of
%          design parameters, configuration (pos,vel,acc) and an external
%          load on the end-effector.
%--------------------------------------------------------------------------

clc;clf
addpath('continuumdynsim');
delta_vec = [0,90,180,270]*pi/180; 
theta_vec = [89.99,60,30,0.01]*pi/180; 
norm_terms = zeros(length(theta_vec),length(delta_vec));
clr = ['r','b','g','k','m','brown'];
%% Parameters
for count1 = 1:length(theta_vec)
    for count2 = 1:length(delta_vec)
        
        L = .05; %distance between each disk
        r = 2.4/1000; %distance between primary and secondary backbone (should be unnecessary)
        delta = delta_vec(count2); %the direction of curvature
        theta_L =  theta_vec(count1); %the magnitude of curvature
        theta_0 = 90*pi/180; %90 degrees is the default state, no curvature
        %bta = 90*pi/180; %beta is angular distance between backbones (should be unnecessary)
        E_p = 41*10^9; % Young's Modulus of primary backbone [N/m^2] %bellow is smc-370 servometer.com
        
        do = .01; % Outer diameter of backbones (rod) [m]
        s_m = L*[1/6:1/6:1]; %location of the disks along the primary backbone      
        r_D = .1; 
        h_D = .03; % radius and height of the disks [m]
        m_D = .1;
        n_D = 6; %number of disks
        rho=6400;
        rho_length = (pi*(do^2)/4)*rho; %density of primary backbone by length
        g = 9.81; % gravitational constant [m/s2]
        grav = [0;-g;0]; %gravity goes perpendicular to the continuum segment
        
        %% Moment of inertial of primary and secondary backbones, about center axis
        I_p = pi*(do^4)/64;
        
        %% Velocity and accleration
        dot_theta_L = pi;
        dot_delta = pi;
        dot_psi = [dot_theta_L;dot_delta];
        
        ddot_theta_L = pi;
        ddot_delta = pi;
        ddot_psi = [ddot_theta_L;ddot_delta];
        
        %% Jacobians
        J_qpsi = get_Jqpsi(r,theta_L,theta_0,delta);
        J_Lpsi = get_Jspsi(L,theta_L,theta_0,delta,L);
        J_omegapsi = get_Jomegapsi(L,theta_L,theta_0,delta,L);
        
        J_xpsi = [J_Lpsi;J_omegapsi];
        
        
        %% Dynamics terms
        [M,G,DOTM,gradT] = get_DYNAMICS(L,r,delta,theta_L,theta_0,dot_theta_L,...
            dot_delta,s_m,r_D,h_D,m_D,n_D,rho_length,...
            E_p,I_p,grav);
        
                 %   G= get_GRAVITY(L,r,L_act,delta,theta_L,theta_0,dot_theta_L,...
                 %    dot_delta,bta,s_m,r_D,h_D,m_D,n_D,rho_length,...
                 %   E_s,E_p,I_s,I_p,grav);
        
        
         XI = ((M.RD + M.TD + M.TPB)*ddot_psi + ...
             (DOTM.RD + DOTM.TD + DOTM.TPB )*dot_psi - ...
             (gradT.RD  + gradT.TD + gradT.TPB) + ...
            (G.D' + G.PB'));
        
%         
%          XI_KE_BACKBONE = (M.TPB + M.TSB)*ddot_psi + (DOTM.TPB + DOTM.TSB)*dot_psi - (gradT.TPB + gradT.TSB)
%        %   XI = XI_KE_BACKBONE
%           XI_KE_DISK = (M.TD + M.RD)*ddot_psi + (DOTM.TD + DOTM.RD)*dot_psi - (gradT.TD + gradT.RD)
% %          XI = XI_KE_DISK
%           XI_GRAV_BACKBONE = G.PB' + G.SB'
% %          XI = XI_GRAV_BACKBONE   
%            XI_GRAV_DISK = G.D'
% %           XI = XI_GRAV_DISK;  
        norm_terms(count1,count2) = norm(XI);
        
        
        %% Wrench Estimation
        n_t = [1 0 0]; % the tissue tangent (opposite to the splipage)
        n_n = [0 1 0]; % the tissue normal
        c_t = 1;
        c_n = 1;
        n_t = n_t';
        n_n = n_n';
        
        Se = [cross(n_t,n_n)*cross(n_t,n_n)',zeros(3,3)
            zeros(3,3),eye(3)];
        Wse = [c_t*n_t+c_n*n_n;zeros(3,1)];
        % Null space projector
        N = (eye(6)-pinv(J_xpsi')*J_xpsi');
        OMEGA = N'*Se*N;
        
        %% Dynamic correction
        %Nullspace projection vector
        eta_corr = pinv(OMEGA) * N' * Se *(Wse - pinv(J_xpsi') * XI);
        w_corr = pinv(J_xpsi') * XI + N * eta_corr;
        
        
        %% External wrench
        
        %w_e = [sqrt(0.5);sqrt(0.5);0;0;0;0];
        % unit Wrench acting along the direction of the correction
        w_e = w_corr/norm(w_corr);
        
        %% Simulated tau assuming statics and dynamics
        tau_statics = pinv(transpose(J_qpsi))*(G.ELASTIC - transpose(J_xpsi)*w_e);
        tau_dyn = pinv(transpose(J_qpsi))*((XI + G.ELASTIC) - transpose(J_xpsi)*w_e);
        
        %% Static force sensing
        %Nullspace projection vector
        eta_static = pinv(OMEGA) * N' * Se *(Wse - pinv(J_xpsi') * (G.ELASTIC  - J_qpsi' * tau_dyn));
        w_ss = pinv(J_xpsi') * (G.ELASTIC  - J_qpsi'*tau_dyn) + N * eta_static;
        w_s = w_ss + w_corr;
        
        %% error
        error_percent(count1,count2) = 100*norm(w_corr)/norm(w_s);
        
        %save( strcat('Data/','L=',num2str(1000*L),'mm','r=',num2str(1000*r),'_gravity_assumed.mat'),...
        %       'error_percent','delta_vec','theta_vec','L','r','rho','rho_steel','E_s','E_p',...
        %       'do','s_m','r_D','h_D','n_D','dot_psi','ddot_psi');
        %% Sanity check
%         if abs(norm(w_e) - norm(w_s)) > 1e-12
%             fprintf('Error! Norms of ground truth and sensed wrench do not match');
%             break;
%         end
        
    end
    
    subplot(1,2,1)
    Fig1 = polarplot(delta_vec,error_percent(count1,:));hold on;
    
    set(Fig1,'Color',clr(count1));
    title('Wrench Estimation error');
    %title(strcat('Bending Angle, \theta_L = ',num2str(theta_vec(count1)*180/pi),' deg'));
    subplot(1,2,2);
    Fig2 = polarplot(delta_vec,norm_terms(count1,:));hold on;
    
    set(Fig2,'Color',clr(count1));
    title('Norm of dynamic term')
    Leg{count1} = ['\theta_L = ' num2str(theta_L*180/pi) 'deg'];
    
end


