function J=jacobian(theta1_val,theta2_val)
 J=[-sin(theta1_val + theta2_val)/2 - sin(theta1_val)/2 -sin(theta1_val + theta2_val)/2 0 0;
     cos(theta1_val + theta2_val)/2+cos(theta1_val)/2 cos(theta1_val + theta2_val)/2 0 0;
        0 0 1 0];   
end
