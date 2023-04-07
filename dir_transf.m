function direct_kinematics=dir_transf(theta1_val,theta2_val,d3_val)
    direct_kinematics=[0.5*cos(theta1_val)+0.5*cos(theta1_val+theta2_val);
        0.5*sin(theta1_val)+0.5*sin(theta1_val+theta2_val);
        d3_val]; % works only for d3 and not 1-d3
end