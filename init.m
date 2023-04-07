clear all
close all
clc
vrclear
vrclose

load('generated_traj.mat');
control;
sim('control.mdl');

tiledlayout(4,4)
nexttile
plot(ans.theta1_generated)
title("theta1 trajectory")
nexttile
plot(ans.theta1_error,"Color","r")
title("Error for theta 1 trajectory")
nexttile
plot(ans.velocity_joints.Data(1,:))
title("theta 1 velocity")
nexttile
plot(ans.velocity_error.Data(1,:),"Color","r")
title("Error for theta 1 velocity")

nexttile
plot(ans.theta2_generated)
title("theta2 trajectory")
nexttile
plot(ans.theta2_error,"Color","r")
title("Error for theta 2 trajectory","r")
nexttile
plot(ans.velocity_joints.Data(2,:))
title("theta 2 velocity")
nexttile
plot(ans.velocity_error.Data(2,:),"Color","r")
title("Error for theta 2 velocity")


nexttile
plot(ans.d3_generated)
title("d3 trajectory")
nexttile
plot(ans.d3_error,"Color","r")
title("Error for d3 trajectory","r")
nexttile
plot(ans.velocity_joints.Data(3,:))
title("d3 velocity")
nexttile
plot(ans.velocity_error.Data(3,:))
title("d3 velocity error")



nexttile
plot(ans.theta4_generated)
title("theta4 trajectory")
nexttile
plot(ans.error_theta4,"Color","r")
title("Error for theta4 trajectory","r")
nexttile
plot(ans.velocity_joints.Data(4,:))
title("theta4 velocity")
nexttile
plot(ans.velocity_error.Data(3,:))
title("theta4 velocity error")





%SCARA_VR_VISUALIZE(squeeze(q(:,1,:)), false);