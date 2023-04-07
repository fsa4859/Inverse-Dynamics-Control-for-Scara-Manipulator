function jDot=jacobian_dot(theta1,theta2,theta1_dot,theta2_dot)
%jDot=[sin(theta1 + theta2)/2 sin(theta1 + theta2)/2 0 0;
    %-cos(theta1 + theta2)/2 -cos(theta1 + theta2)/2 0 0;
    %0 0 0 0];% using one diff
 
 %jDot=[- cos(theta1 + theta2) - cos(theta1)/2 -cos(theta1 + theta2) 0 0;
     %- sin(theta1 + theta2) - sin(theta1)/2 -sin(theta1 + theta2) 0 0;
     %0 0 0 0];

 jDot=[-0.5*cos(theta1)*theta1_dot-0.5*(cos(theta1+theta2)*(theta1_dot+theta2_dot)) -0.5*cos(theta1+theta2)*(theta1_dot+theta2_dot) 0 0;
     -0.5*sin(theta1)*theta1_dot-0.5*(sin(theta1+theta2)*(theta1_dot+theta2_dot)) -0.5*(sin(theta1+theta2)*(theta1_dot+theta2_dot)) 0 0;
     0 0 0 0];

end

