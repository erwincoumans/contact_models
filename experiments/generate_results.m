clear
close all

figure('Position',[100 100 333 256])
exp_sliding
f = figure();
plot_bead(params, st4(:,21), x4(:,21))
f.Position(1:2) = [100 450];

figure('Position',[450 100 333 256])
exp_wedging
f = figure();
plot_box(params, st4(:,201), x4(:,201))
f.Position(1:2) = [450 450];

figure('Position',[800 100 333 256])
exp_grasping
f = figure();
plot_gripper(params, st4(:,17), x4(:,17))
f.Position(1:2) = [800 450];

exp_stacking

if (exist('lemke','file') == 2)
    figure('Position',[1500 450 333 256])
    exp_stacking_err
end