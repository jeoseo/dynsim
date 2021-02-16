function C=ComputeC(th1,th2,th3,thd1,thd2,thd3)
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
    
C1=[- thd2*((m3*sin(2*th2)*l1^2)/2 + m3*sin(2*th2 + th3)*l1*r2 + (m2*sin(2*th2)*r1^2)/2 + (m3*sin(2*th2 + 2*th3)*r2^2)/2 - (Iyy3*sin(2*th2 + 2*th3))/2 + (Izz3*sin(2*th2 + 2*th3))/2 - (Iyy2*sin(2*th2))/2 + (Izz2*sin(2*th2))/2) - thd3*sin(th2 + th3)*(m3*cos(th2 + th3)*r2^2 + l1*m3*cos(th2)*r2 - Iyy3*cos(th2 + th3) + Izz3*cos(th2 + th3)), -thd1*((m3*sin(2*th2)*l1^2)/2 + m3*sin(2*th2 + th3)*l1*r2 + (m2*sin(2*th2)*r1^2)/2 + (m3*sin(2*th2 + 2*th3)*r2^2)/2 - (Iyy3*sin(2*th2 + 2*th3))/2 + (Izz3*sin(2*th2 + 2*th3))/2 - (Iyy2*sin(2*th2))/2 + (Izz2*sin(2*th2))/2), -thd1*sin(th2 + th3)*(m3*cos(th2 + th3)*r2^2 + l1*m3*cos(th2)*r2 - Iyy3*cos(th2 + th3) + Izz3*cos(th2 + th3))];
C2=[                                                                                                                 thd1*((m3*sin(2*th2)*l1^2)/2 + m3*sin(2*th2 + th3)*l1*r2 + (m2*sin(2*th2)*r1^2)/2 + (m3*sin(2*th2 + 2*th3)*r2^2)/2 - (Iyy3*sin(2*th2 + 2*th3))/2 + (Izz3*sin(2*th2 + 2*th3))/2 - (Iyy2*sin(2*th2))/2 + (Izz2*sin(2*th2))/2),                                                                                                                                                                                                      -l1*m3*r2*thd3*sin(th3),                                                                              -l1*m3*r2*sin(th3)*(thd2 + thd3)];
C3=[                                                                                                                                                                                                                                thd1*sin(th2 + th3)*(m3*cos(th2 + th3)*r2^2 + l1*m3*cos(th2)*r2 - Iyy3*cos(th2 + th3) + Izz3*cos(th2 + th3)),                                                                                                                                                                                                       l1*m3*r2*thd2*sin(th3),                                                                                                             0];
    C=[C1;C2;C3];
end