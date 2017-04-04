function ForwKin = forwardKinematics(params, theta)
    L1=params(3);
    L2=params(4);

    x1=L1.*sin(theta(:,1)); 
    y1=L1.*cos(theta(:,1)); 

    x2=L1.*sin(theta(:,1))+L2.*sin(theta(:,1)+theta(:,2)); 
    y2=L1.*cos(theta(:,1))+L2.*cos(theta(:,1)+theta(:,2)); 

    ForwKin=[x1, y1, x2, y2];
end