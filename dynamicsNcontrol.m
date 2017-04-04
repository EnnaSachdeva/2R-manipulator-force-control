function xdot=dynamicsNcontrol(t,x,thFin,params,Kpid)
    xdot=zeros(8,1);

    %% Final angle
    thFin1=thFin(1);
    thFin2=thFin(2);


    %% Robot Specifications
    M1=params(1);
    M2=params(2);
    L1=params(3);
    L2=params(4);
    g=9.8;

    %% Inertia Matrix
    Bq=[(M1+M2)*L1^2+M2*L2^2+2*M2*L1*L2*cos(x(4))   M2*L2^2+M2*L1*L2*cos(x(4));
         M2*L2^2+M2*L1*L2*cos(x(4))                 M2*L2^2];

    %% C Matrix
    Cq=[-M2*L1*L2*sin(x(4))*(2*x(5)*x(6)+x(6)^2);
        -M2*L1*L2*sin(x(4))*x(5)*x(6)];

    %% Gravity Matrix
    Gq=[-(M1+M2)*g*L1*sin(x(3))-M2*g*L2*sin(x(3)+x(4));
        -M2*g*L2*sin(x(3)+x(4))];

    Ftelda=PID(Kpid,x,thFin); 
    F=Bq*Ftelda;  

    %% Feedback linearization
    qDotDot=feedbackLinear(Ftelda,Bq,Cq,Gq);

    %STATES
    xdot=[ thFin1-x(3);    % error1
           thFin2-x(4);    % error2
           x(5);         % thetaDot1
           x(6);         % thetaDot2    
           qDotDot(1);   % thetaDotDot1
           qDotDot(2);  % thetaDotDot2
            F(1);         % F1
            F(2)];        % F2

end