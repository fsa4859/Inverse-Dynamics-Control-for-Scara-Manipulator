function J_Inv=jacobian_inverse(theta1,theta2)
  J_Inv=[(2*cos(theta1 + theta2))/sin(theta2) (2*sin(theta1 + theta2))/sin(theta2) 0;
      -(2*cos(theta1 + theta2) + 2*cos(theta1))/sin(theta2) -(2*sin(theta1 + theta2) + 2*sin(theta1))/sin(theta2) 0;
      0 0 1;0 0 0]; 
end
