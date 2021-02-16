function EOM=ComputeEOM(th1,th2,th3,thd1,thd2,thd3,thdd1,thdd2,thdd3)
        Izz1=1;
    Izz2=1;
    Izz3=1;
        Ixx1=1;
    Ixx2=1;
    Ixx3=1;
        Iyy1=1;
    Iyy2=1;
    Iyy3=1;
    r1=1;
    r2=1;
    r3=1;
    m1=1;
    m2=1;
    m3=1;
    l1=1;
    l2=1;
    l3=1;
    
    D=ComputeD(th1,th2,th3);
    C=ComputeC(th1,th2,th3,thd1,thd2,thd3);
    
    g=[0; -(m2*9.81*r1+m3*9.81*l1)*cos(th2)-m3*r2*cos(th2+th3);-m3*9.81*r2*cos(th2+th3)];
    EOM=D*[thdd1;thdd2;thdd3]+C*[thd1;thd2;thd3]+g;
end