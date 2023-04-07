function n = N_Matrix(theta2,qdot_1,qdot_2,qdot_3,qdot_4)
    n=[qdot_1/10000 - (45*qdot_2^2*sin(theta2))/8 - (45*qdot_1*qdot_2*sin(theta2))/4;
        (45*sin(theta2)*qdot_1^2)/8 + qdot_2/10000;
        qdot_3/2 + 2943/20;
        qdot_4/10];
end