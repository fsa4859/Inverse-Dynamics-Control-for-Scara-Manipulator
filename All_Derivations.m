syms theta1 theta2 d3 theta4
% calculate values for link 1
ml1=25;
Il1=diag([5 5 5]);
Im1=diag([0.0001 0.0001 0.0001]);
% initialize jpl1
% only first column of jpl1 is filled
z0=[0;0;1];
p0=[0;0;1];
pl1=[0.25*cos(theta1);0.25*sin(theta1);0];
rev_pl1=cross(z0,pl1-p0);
% find jacobian position
jpl1=[rev_pl1(1) 0 0 0;
        rev_pl1(2) 0 0 0;
        rev_pl1(3) 0 0 0;]; % correct
% find jacobian orientation
jol1=[0 0 0 0;0 0 0 0;1 0 0 0]; % correct
% finding rotation matrix
R1=[cos(theta1) -sin(theta1) 0;
    sin(theta1) cos(theta1) 0;
    0 0 1]; % correct
% motor position
jpm1=zeros(3,4); % correct
% orientation of the motor
jom1=[0 0 0 0;0 0 0 0;1 0 0 0]; % correct
% rotation of motor
Rm1=[1 0 0;0 1 0;0 0 1]; % correct (identity matrix) Rm1=R0
% find B for link 1
B_1=ml1*transpose(jpl1)*jpl1+transpose(jol1)*R1*Il1*transpose(R1)*jol1...
    +transpose(jpm1)*jpm1+transpose(jom1)*Rm1*Im1*transpose(Rm1)*jom1;
B_1=simplify(B_1);


%%%%%%%%%%%%%%%%%%% link2 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
pl2=[0.5*cos(theta1)+0.25*(cos(theta1+theta2));
    0.5*sin(theta1)+0.25*sin(theta1+theta2);0]; % correcy
Il2=diag([5 5 5]);
Im2=diag([0.0001 0.0001 0.0001]);
z1=[0;0;1];
p1=[0.5*cos(theta1);0.5*sin(theta1);0];
ml2=25
jp1_l2=cross(z0,pl2-p0);
jp2_l2=cross(z1,pl2-p1);

jpl2=[jp1_l2 jp2_l2 [0;0;0] [0;0;0]]; % correct

% orientation jacobian
jol2=[[0;0;1] [0;0;1] [0;0;0] [0;0;0]]; % correct

R2=[cos(theta1)*cos(theta2)-sin(theta1)*sin(theta2) -cos(theta1)*sin(theta2)-cos(theta2)*sin(theta1) 0;
    cos(theta1)*sin(theta2)+cos(theta2)*sin(theta1) cos(theta1)*cos(theta2)-sin(theta2)*sin(theta1) 0;
    0 0 1]; % this is the correct one

% jacobian position for motor
pm2=[0.5*cos(theta1);0.5*sin(theta1);1]; % correct

% jpm2
jpm2=[cross(z1,pm2-p1) [0;0;0] [0;0;0] [0;0;0]];
jpm2_trial=[-0.5*sin(theta1) 0 0 0;0.5*cos(theta1) 0 0 0;0 0 0 0]; % correct

%orinetation component
jo_m2=[[0;0;1] [0;0;1] [0;0;0] [0;0;0]]; % correct

Rm2=[cos(theta1) -sin(theta1) 0;sin(theta1) cos(theta1) 0;0 0 1]; % correct

B_2=ml2*transpose(jpl2)*jpl2+transpose(jol2)*R2*Il2*transpose(R2)*jol2....
    +transpose(jpm2_trial)*jpm2_trial+transpose(jo_m2)*Rm2*Im2*transpose(Rm2)*jo_m2;
B_2=simplify(B_2);

b11=5+25*0.25^2+1*0.0001^2+5+25*(0.5^2+0.25^2+2*0.5*0.25*cos(theta2))+0.0001+0.5^2;
b12=5+25*(0.25^2+0.5*0.25*cos(theta2))+0.0001;
b22=5+25*0.25^2+1*0.0001;

B_text_book_planararm=[b11 b12 0 0;b12 b22 0 0;0 0 0 0;0 0 0 0];

% completing the second part from textbook
b11_second=10*(0.5^2+0.5^2+2*0.5*0.5*cos(theta2))+1;
b12_second=10*(0.5^2+0.5*0.5*cos(theta2))+1;
b14_second=1;
b22_second=1+10*0.5^2;
b24_second=1;
b33_second=10;
b44_second=1;

B_second=[b11_second b12_second 0 b14_second;
    b12_second b22_second 0 b24_second;
    0 0 b33_second 0;
    b14_second b24_second 0 b44_second];

B_total_updated=simplify(B_text_book_planararm+B_second);

B_total_inverse_updated=inv(B_total_updated);

B_total_inverse=simplify(B_total_inverse_updated);

syms qdot_1 qdot_2 qdot_3 qdot_4
qdot=[qdot_1;qdot_2;qdot_3;qdot_4];

% values for c11
c111=simplify(0.5*(diff(B_total_updated(1,1),theta1) + diff(B_total_updated(1,1),theta1)-diff(B_total_updated(1,1),theta1)));
c112=simplify(0.5*(diff(B_total_updated(1,1),theta2) + diff(B_total_updated(1,2),theta1)-diff(B_total_updated(1,2),theta1)));
c113=simplify(0.5*(diff(B_total_updated(1,1),d3) + diff(B_total_updated(1,3),theta1)-diff(B_total_updated(1,3),theta1)));
c114=simplify(0.5*(diff(B_total_updated(1,1),theta4) + diff(B_total_updated(1,4),theta1)-diff(B_total_updated(1,4),theta1)));
c11=c111*qdot_1+c112*qdot_2+c113*qdot_3+c114*qdot_4;

% values for c12
c121=simplify(0.5*(diff(B_total_updated(1,2),theta1) + diff(B_total_updated(1,1),theta2)-diff(B_total_updated(2,1),theta1)));
c122=simplify(0.5*(diff(B_total_updated(1,2),theta2) + diff(B_total_updated(1,2),theta2)-diff(B_total_updated(2,2),theta1)));
c123=simplify(0.5*(diff(B_total_updated(1,2),d3) + diff(B_total_updated(1,3),theta2)-diff(B_total_updated(2,3),theta1)));
c124=simplify(0.5*(diff(B_total_updated(1,2),theta4) + diff(B_total_updated(1,4),theta2)-diff(B_total_updated(2,4),theta1)));
c12=c121*qdot_1+c122*qdot_2+c123*qdot_3+c124*qdot_4;

% values for c13
c131=simplify(0.5*(diff(B_total_updated(1,3),theta1) + diff(B_total_updated(1,1),d3)-diff(B_total_updated(3,1),theta1)));
c132=simplify(0.5*(diff(B_total_updated(1,3),theta2) + diff(B_total_updated(1,2),d3)-diff(B_total_updated(3,2),theta1)));
c133=simplify(0.5*(diff(B_total_updated(1,3),d3) + diff(B_total_updated(1,3),d3)-diff(B_total_updated(3,3),theta1)));
c134=simplify(0.5*(diff(B_total_updated(1,3),theta4) + diff(B_total_updated(1,4),d3)-diff(B_total_updated(3,4),theta1)));
c13=c131*qdot_1+c132*qdot_2+c133*qdot_3+c134*qdot_4;


% values for c14
c141=simplify(0.5*(diff(B_total_updated(1,4),theta1) + diff(B_total_updated(1,1),theta4)-diff(B_total_updated(4,1),theta1)));
c142=simplify(0.5*(diff(B_total_updated(1,4),theta2) + diff(B_total_updated(1,2),theta4)-diff(B_total_updated(4,2),theta1)));
c143=simplify(0.5*(diff(B_total_updated(1,4),d3) + diff(B_total_updated(1,3),theta4)-diff(B_total_updated(4,3),theta1)));
c144=simplify(0.5*(diff(B_total_updated(1,4),theta4) + diff(B_total_updated(1,4),theta4)-diff(B_total_updated(4,4),theta1)));
c14=c141*qdot_1+c142*qdot_2+c143*qdot_3+c144*qdot_4;


% values for c21
c211=simplify(0.5*(diff(B_total_updated(2,1),theta1) + diff(B_total_updated(2,1),theta1)-diff(B_total_updated(1,1),theta2)));
c212=simplify(0.5*(diff(B_total_updated(2,1),theta2) + diff(B_total_updated(2,2),theta1)-diff(B_total_updated(1,2),theta2)));
c213=simplify(0.5*(diff(B_total_updated(2,1),d3) + diff(B_total_updated(2,3),theta1)-diff(B_total_updated(1,3),theta2)));
c214=simplify(0.5*(diff(B_total_updated(2,1),theta4) + diff(B_total_updated(2,4),theta1)-diff(B_total_updated(1,4),theta2)));
c21=c211*qdot_1+c212*qdot_2+c213*qdot_3+c214*qdot_4;


% values for c22
c221=simplify(0.5*(diff(B_total_updated(2,2),theta1) + diff(B_total_updated(2,1),theta2)-diff(B_total_updated(2,1),theta2)));
c222=simplify(0.5*(diff(B_total_updated(2,2),theta2) + diff(B_total_updated(2,2),theta2)-diff(B_total_updated(2,2),theta2)));
c223=simplify(0.5*(diff(B_total_updated(2,2),d3) + diff(B_total_updated(2,3),theta2)-diff(B_total_updated(2,3),theta2)));
c224=simplify(0.5*(diff(B_total_updated(2,2),theta4) + diff(B_total_updated(2,4),theta2)-diff(B_total_updated(2,4),theta2)));
c22=c221*qdot_1+c222*qdot_2+c223*qdot_3+c224*qdot_4;


% values for c23
c231=simplify(0.5*(diff(B_total_updated(2,3),theta1) + diff(B_total_updated(2,1),d3)-diff(B_total_updated(3,1),theta2)));
c232=simplify(0.5*(diff(B_total_updated(2,3),theta2) + diff(B_total_updated(2,2),d3)-diff(B_total_updated(3,2),theta2)));
c233=simplify(0.5*(diff(B_total_updated(2,3),d3) + diff(B_total_updated(2,3),d3)-diff(B_total_updated(3,3),theta2)));
c234=simplify(0.5*(diff(B_total_updated(2,3),theta4) + diff(B_total_updated(2,4),d3)-diff(B_total_updated(3,4),theta2)));
c23=c231*qdot_1+c232*qdot_2+c233*qdot_3+c234*qdot_4;


% values for c24
c241=simplify(0.5*(diff(B_total_updated(2,4),theta1) + diff(B_total_updated(2,1),theta4)-diff(B_total_updated(4,1),theta2)));
c242=simplify(0.5*(diff(B_total_updated(2,4),theta2) + diff(B_total_updated(2,2),theta4)-diff(B_total_updated(4,2),theta2)));
c243=simplify(0.5*(diff(B_total_updated(2,4),d3) + diff(B_total_updated(2,3),theta4)-diff(B_total_updated(4,3),theta2)));
c244=simplify(0.5*(diff(B_total_updated(2,4),theta4) + diff(B_total_updated(2,4),theta4)-diff(B_total_updated(4,4),theta2)));
c24=c241*qdot_1+c242*qdot_2+c243*qdot_3+c244*qdot_4;

% values for c31
c311=simplify(0.5*(diff(B_total_updated(3,1),theta1) + diff(B_total_updated(3,1),theta1)-diff(B_total_updated(1,1),d3)));
c312=simplify(0.5*(diff(B_total_updated(3,1),theta2) + diff(B_total_updated(3,2),theta1)-diff(B_total_updated(1,2),d3)));
c313=simplify(0.5*(diff(B_total_updated(3,1),d3) + diff(B_total_updated(3,3),theta1)-diff(B_total_updated(1,3),d3)));
c314=simplify(0.5*(diff(B_total_updated(3,1),theta4) + diff(B_total_updated(3,4),theta1)-diff(B_total_updated(1,4),d3)));
c31=c311*qdot_1+c312*qdot_2+c313*qdot_3+c314*qdot_4;

% values for c32
c321=simplify(0.5*(diff(B_total_updated(3,2),theta1) + diff(B_total_updated(3,1),theta2)-diff(B_total_updated(2,1),d3)));
c322=simplify(0.5*(diff(B_total_updated(3,2),theta2) + diff(B_total_updated(3,2),theta2)-diff(B_total_updated(2,2),d3)));
c323=simplify(0.5*(diff(B_total_updated(3,2),d3) + diff(B_total_updated(3,3),theta2)-diff(B_total_updated(2,3),d3)));
c324=simplify(0.5*(diff(B_total_updated(3,2),theta4) + diff(B_total_updated(3,4),theta2)-diff(B_total_updated(2,4),d3)));
c32=c321*qdot_1+c322*qdot_2+c323*qdot_3+c324*qdot_4;

% values for c33
c331=simplify(0.5*(diff(B_total_updated(3,3),theta1) + diff(B_total_updated(3,1),d3)-diff(B_total_updated(3,1),d3)));
c332=simplify(0.5*(diff(B_total_updated(3,3),theta2) + diff(B_total_updated(3,2),d3)-diff(B_total_updated(3,2),d3)));
c333=simplify(0.5*(diff(B_total_updated(3,3),d3) + diff(B_total_updated(3,3),d3)-diff(B_total_updated(3,3),d3)));
c334=simplify(0.5*(diff(B_total_updated(3,3),theta4) + diff(B_total_updated(3,4),d3)-diff(B_total_updated(3,4),d3)));
c33=c331*qdot_1+c332*qdot_2+c333*qdot_3+c334*qdot_4;


% values for c34
c341=simplify(0.5*(diff(B_total_updated(3,4),theta1) + diff(B_total_updated(3,1),theta4)-diff(B_total_updated(4,1),d3)));
c342=simplify(0.5*(diff(B_total_updated(3,4),theta2) + diff(B_total_updated(3,2),theta4)-diff(B_total_updated(4,2),d3)));
c343=simplify(0.5*(diff(B_total_updated(3,4),d3) + diff(B_total_updated(3,3),theta4)-diff(B_total_updated(4,3),d3)));
c344=simplify(0.5*(diff(B_total_updated(3,4),theta4) + diff(B_total_updated(3,4),theta4)-diff(B_total_updated(4,4),d3)));
c34=c341*qdot_1+c342*qdot_2+c343*qdot_3+c344*qdot_4;


% values for c41
c411=simplify(0.5*(diff(B_total_updated(4,1),theta1) + diff(B_total_updated(4,1),theta1)-diff(B_total_updated(1,1),theta4)));
c412=simplify(0.5*(diff(B_total_updated(4,1),theta2) + diff(B_total_updated(4,2),theta1)-diff(B_total_updated(1,2),theta4)));
c413=simplify(0.5*(diff(B_total_updated(4,1),d3) + diff(B_total_updated(4,3),theta1)-diff(B_total_updated(1,3),theta4)));
c414=simplify(0.5*(diff(B_total_updated(4,1),theta4) + diff(B_total_updated(4,4),theta1)-diff(B_total_updated(1,4),theta4)));
c41=c411*qdot_1+c412*qdot_2+c413*qdot_3+c414*qdot_4;


% values for c42
c421=simplify(0.5*(diff(B_total_updated(4,2),theta1) + diff(B_total_updated(4,1),theta2)-diff(B_total_updated(2,1),theta4)));
c422=simplify(0.5*(diff(B_total_updated(4,2),theta2) + diff(B_total_updated(4,2),theta2)-diff(B_total_updated(2,2),theta4)));
c423=simplify(0.5*(diff(B_total_updated(4,2),d3) + diff(B_total_updated(4,3),theta2)-diff(B_total_updated(2,3),theta4)));
c424=simplify(0.5*(diff(B_total_updated(4,2),theta4) + diff(B_total_updated(4,4),theta2)-diff(B_total_updated(2,4),theta4)));
c42=c421*qdot_1+c422*qdot_2+c423*qdot_3+c424*qdot_4;

% values for c43
c431=simplify(0.5*(diff(B_total_updated(4,3),theta1) + diff(B_total_updated(4,1),d3)-diff(B_total_updated(3,1),theta4)));
c432=simplify(0.5*(diff(B_total_updated(4,3),theta2) + diff(B_total_updated(4,2),d3)-diff(B_total_updated(3,2),theta4)));
c433=simplify(0.5*(diff(B_total_updated(4,3),d3) + diff(B_total_updated(4,3),d3)-diff(B_total_updated(3,3),theta4)));
c434=simplify(0.5*(diff(B_total_updated(4,3),theta4) + diff(B_total_updated(4,4),d3)-diff(B_total_updated(3,4),theta4)));
c43=c431*qdot_1+c432*qdot_2+c433*qdot_3+c434*qdot_4;

% values for c44
c441=simplify(0.5*(diff(B_total_updated(4,4),theta1) + diff(B_total_updated(4,1),theta4)-diff(B_total_updated(4,1),theta4)));
c442=simplify(0.5*(diff(B_total_updated(4,4),theta2) + diff(B_total_updated(4,2),theta4)-diff(B_total_updated(4,2),theta4)));
c443=simplify(0.5*(diff(B_total_updated(4,4),d3) + diff(B_total_updated(4,3),theta4)-diff(B_total_updated(4,3),theta4)));
c444=simplify(0.5*(diff(B_total_updated(4,4),theta4) + diff(B_total_updated(4,4),theta4)-diff(B_total_updated(4,4),theta4)));
c44=c441*qdot_1+c442*qdot_2+c443*qdot_3+c444*qdot_4;

C=[c11 c12 c13 c14;
    c21 c22 c23 c24;
    c31 c32 c33 c34;
    c41 c42 c43 c44];

G_new=[0;0;10*9.81;0];

F=diag([1*0.0001 1*0.0001 50*0.01 20*0.005]);

jac=[-sin(theta1 + theta2)/2 - sin(theta1)/2 -sin(theta1 + theta2)/2 0 0;
     cos(theta1 + theta2)/2+cos(theta1)/2 cos(theta1 + theta2)/2 0 0;
        0 0 1 0];   
jac_transpose=transpose(jac)
force_end_effector=[0;0;-5*9.81];

% translate the force to the joint space
joint_torque=jac_transpose*force_end_effector;


% get n(q,qdot)
n=simplify(C*qdot+F*qdot+G_new-joint_torque);


%{
%%%%%%%%%%%%%%%%%%%% link 3 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
ml3=10;
Im3=diag([0.01 0.01 0.01]);
z2=[0;0;1];
jpl3=[[0;0;1] [0;0;1] [0;0;1] [0;0;0]];
R3=[cos(theta1)*cos(theta2)- sin(theta1)*sin(theta2), - cos(theta1)*sin(theta2) - cos(theta2)*sin(theta1),0;
    cos(theta1)*sin(theta2) + cos(theta2)*sin(theta1),cos(theta1)*cos(theta2) - sin(theta1)*sin(theta2),0;
    0,0,1];

% position jacobian of the motor
jpm3=[z1 z2 [0;0;0] [0;0;0]];
jom3=[[0;0;0] [0;0;0] 50*[0;0;1] [0;0;0]];

B_3=ml3*transpose(jpl3)*jpl3+transpose(jpm3)*jpm3+transpose(jom3)*R2*Im3*transpose(R2)*(jom3);

%%%%%%%%%%%% link 4 %%%%%%%%%%%%%%%%%%%
p3=[cos(theta1)/2 +0.5*cos(theta1+theta2);
    sin(theta1)/2 + 0.5*sin(theta1+theta2);
     d3];
Il4=diag([1 1 1]);
Im4=diag([0.005 0.005 0.005]);

p=[0.5*(cos(theta1 + theta2)/4 + cos(theta1)/2);
    0.5*(sin(theta1 + theta2)/4 + sin(theta1)/2);
    0.5*(1-d3)];

pl4=p3+p;

p2=[cos(theta1 + theta2)/4 + cos(theta1)/2;
    sin(theta1 + theta2)/4 + sin(theta1)/2;
    1];

z3=[0;0;1];

jpl4=[cross(z0,pl4-p0) cross(z1,pl4-p1) cross(z2,pl4-p2) cross(z3,pl4-p3)];
jpl4=simplify(jpl4)

jol4=[z0 z1 z2 z3];

R4=[cos(theta1 + theta2 + theta4) -sin(theta1 + theta2 + theta4) 0;
    sin(theta1 + theta2 + theta4) cos(theta1 + theta2 + theta4) 0;
    0 0 1];

jp_m4=[cross(z0,p3-p0) cross(z1,p3-p1) cross(z2,p3-p2) [0;0;0]];

jo_m4=[z0 z1 z2 20*z3];

B_4=transpose(jpl4)*jpl4+transpose(jol4)*R4*Il4*transpose(R4)*jol4...
    +transpose(jp_m4)*jp_m4+transpose(jo_m4)*R3*Im4*transpose(R3)*jo_m4;


B_total=simplify(B_1+B_2+B_3+B_4);
inv_B_total=simplify(inv(B_total));

B_planar_arm=[25+0.0001+25 0 0 0;
    0 25+0.0001 0 0;
    0 0 0 0;
    0 0 0 0];
jpl4_new=[-0.5*sin(theta1)-0.5*sin(theta1+theta2) -0.5*sin(theta1+theta2) 0 0;
    0.5*cos(theta1)+0.5*cos(theta1+theta2) 0.5*cos(theta1+theta2) 0 0;
    0 0 1 0];
jol4_new=[0 0 0 0;0 0 0 0;1 1 0 1];
B_new=B_planar_arm+10*transpose(jpl4_new)*jpl4_new+transpose(jol4_new)*R4*transpose(R4)*jol4_new;
B_new=simplify(B_new);
inv_b_new=inv(B_new);
inv_b_new=subs(inv_b_new)

%B_total_derivative=diff(B_total,theta1)+diff(B_total,theta2)...
    %+diff(B_total,d3)+diff(B_total,theta4);
q=[theta1;theta2;d3;theta4];

% calculating C
% calculation for c11



% calculating G term in dynamic model
g=[0 0 -9.81];
gl1=simplify(-ml1*g*jpl1(:,1)-ml2*g*jpl2(:,1)....
    -ml3*g*jpl3(:,1)-1*g*jpl4(:,1));
gm1=simplify(1*g*jpm1(:,1)-1*g*jpm2(:,1)....
    -1*g*jpm3(:,1)-1*g*jp_m4(:,1));

gl2=simplify(-ml1*g*jpl1(:,2)-ml2*g*jpl2(:,2)....
    -ml3*g*jpl3(:,2)-1*g*jpl4(:,2));
gm2=simplify(1*g*jpm1(:,2)-1*g*jpm2(:,2)....
    -1*g*jpm3(:,2)-1*g*jp_m4(:,2));

gl3=simplify(-ml1*g*jpl1(:,3)-ml2*g*jpl2(:,3)....
    -ml3*g*jpl3(:,3)-1*g*jpl4(:,3));
gm3=simplify(1*g*jpm1(:,3)-1*g*jpm2(:,3)....
    -1*g*jpm3(:,3)-1*g*jp_m4(:,3));

gl4=simplify(-ml1*g*jpl1(:,4)-ml2*g*jpl2(:,4)....
    -ml3*g*jpl3(:,4)-1*g*jpl4(:,4));
gm4=simplify(1*g*jpm1(:,4)-1*g*jpm2(:,4)....
    -1*g*jpm3(:,4)-1*g*jp_m4(:,4));

G=[gl1+gm1;gl2+gm2;gl3+gm3;gl4+gm4];
%G_new=[0;0;10*9.81;0];

% compute the F matrix
F=diag([1*0.0001 1*0.0001 50*0.01 20*0.005]);

% compute torque


% get n(q,qdot)
n=simplify(C*qdot+F*qdot+G_new)

%}






