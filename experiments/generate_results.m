clear
close all

figure('Position',[100 100 333 256])
exp_sliding
f = figure();
plot_bead(r, w, x4(:,21), f4(:,21))
f.Position(1:2) = [100 450];

figure('Position',[450 100 333 256])
exp_wedging
f = figure();
plot_box(params,x4(:,201),f4(:,201))
f.Position(1:2) = [450 450];

figure('Position',[800 100 333 256])
exp_grasping
f = figure();
plot_gripper(r, x4(:,17), f4(:,17))
f.Position(1:2) = [800 450];