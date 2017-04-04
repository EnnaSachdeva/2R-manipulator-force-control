function f=PID(Kpid,x,thFin)
    Kp1=Kpid(1);
    Kd1=Kpid(2);
    Ki1=Kpid(3);
    Kp2=Kpid(4);
    Kd2=Kpid(5);
    Ki2=Kpid(6);

    errInt1=x(1);
    errInt2=x(2);

    th1=x(3);
    th2=x(4);
    errth1=thFin(1)-th1;
    errth2=thFin(2)-th2;

    thDot1=x(5);
    thDot2=x(6);

    f1=Kp1*errth1-Kd1*thDot1+Ki1*errInt1;
    f2=Kp2*errth2-Kd2*thDot2+Ki2*errInt2;
    f=[f1;f2];
end