function D=ComputeD(th1,th2,th3)
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
    
D1=[Izz1 + m3*(r2*cos(th2 + th3) + l1*cos(th2))^2 + Izz3*cos(th2 + th3)^2 + Iyy3*sin(th2 + th3)^2 + Izz2*cos(th2)^2 + Iyy2*sin(th2)^2 + m2*r1^2*cos(th2)^2,                                                               0,                               0];
D2=[                                                                                                                                                     0, m3*l1^2 + 2*m3*cos(th3)*l1*r2 + m2*r1^2 + m3*r2^2 + Ixx2 + Ixx3, Ixx3 + m3*r2*(r2 + l1*cos(th3))];
D3=[                                                                                                                                                     0,                                 Ixx3 + m3*r2*(r2 + l1*cos(th3)),                  m3*r2^2 + Ixx3];
 D=[D1;D2;D3];
end