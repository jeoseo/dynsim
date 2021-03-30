%Assume trajectory data has already been collected or simulated with SimTLP.m, 
%Additionally, assume that known variables already populated by SimTLP
%specifically we need [t, js] from the ODE solver, as well as 

%Define unknown physical properties of the robot
%In this case, we pretend that we know the everything but the inertia
%tensor, but we know that the inertia tensor is diagonal.
DOF=2;
It=sym('It',[DOF,3]);
assume(It,'real');
I=sym(I);
I(:,:,1)=diag(It(1,:));
I(:,:,2)=diag(It(2,:));
fv=sym('fv',[DOF,1]);
derive=false; %don't generate functions

th=sym('th',[DOF,1]);
thd=sym('thd',[DOF,1]);
thdd=sym('thdd',[DOF,1]);
assume(th,'real');
assume(thd,'real');
assume(thdd,'real');
J=DeriveBodyJacobians(DOF,q,w,g,derive);
D=DeriveD(J, I,m, DOF,derive);
C=DeriveC(D,DOF,derive);
gth=DeriveFK(DOF,g,w,q,derive);
N=DeriveN(DOF,gth,gravity,fc,fv,derive); 
%We instead need to find a different EOM representation
eqom=D*thdd+C*thd+N;
eqom=simplify(expand(eqom));

coeffCell=cell(1,DOF);
for i=1:DOF
    [coeff,term]=coeffs(eqom(i),{It,fv});
    term=arrayfun(@char, term,'uniform',0); %turn from symbolic to cell
    coeff=arrayfun(@char,coeff,'uniform',0);
    coeffCell{i}=containers.Map(term,coeff);
end
% creating the vector with the different polynomial combianations of the 
% unknown inertial parameters
allTerms=containers.Map('KeyType','char', 'ValueType','any');
for i=1:DOF
    keyList=keys(coeffCell{i});
    for j=1:size(keyList,2)
        if ~isKey(allTerms,keyList{j})
            allTerms(keyList{j})=0; %val in this map is meaningless
        end
    end
end
allKeyTerms=keys(allTerms);
%Create phi, the matrix that converts inertial parameters to joint torques
phi=zeros(DOF,size(allKeyTerms,2));
phi=sym(phi);
for i=1:DOF
    for j=1:size(keys(allTerms),2)
        if isKey(coeffCell{i},allKeyTerms(j))
            phi(i,j)=str2sym(convertCharsToStrings(coeffCell{i}(allKeyTerms{j})));
        end
    end
end
phi_big=[];
tau_phi=[];%tau with the elements matching phiBig
for i=2:10:size(js,2)-1 %trimmed a little so we can get thdd,also downsampled
    phi_sub=phi;
    for j=1:DOF %substitute values for the joint states into phi
        phi_sub=subs(phi_sub,th(j),js(j,i));
        phi_sub=subs(phi_sub,thd(j),js(DOF+j,i));
        phi_sub=subs(phi_sub,thdd(j),(js(DOF+j,i+1)-js(DOF+j,i-1))/(t(i+1)-t(i-1)));        
    end
    phi_sub=double(phi_sub); %phi should now be free of any symbolic vars
    tau_phi=[tau_phi;tau(:,i)];
    phi_big=[phi_big;phi_sub];
end
parameter_val=lsqr(phi_big,tau_phi);
inertial_parameter_map=containers.Map(allKeyTerms,parameter_val);






