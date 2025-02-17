close all;
clear all;
clc;

set(0, 'DefaultAxesFontSize', 13);
set(0, 'DefaultFigurePosition', [0,0,1080,720]);

% 3D trajectory
syms s t;

% x y z
vias = [
  0 0 0;
  1 0 0;
  2 1 0;
  2 1 2;
  2 0 2;
];

circCenters = [
  1 1 0;
  2 1 1;
];

%% 3D trajectory with motion primitives

linear1 = @(u) repmat(vias(1,:)',1,size(u, 2)) + (vias(2,:) - vias(1,:))'*u;
linear4 = @(u) repmat(vias(4,:)',1,size(u, 2)) + (vias(5,:) - vias(4,:))'*u;

R2 = eul2rotm([0, pi, pi],"XYZ");
t2 = [1 1 0];

circ2 = @(u) repmat(t2',1,size(u,2)) + R2 * padarray([cos(u*(-pi/2)+(pi/2))' sin(u*(-pi/2)+(pi/2))'], [0 1], 0, 'post')';

R3 = eul2rotm([0, -pi/2, -pi/2],"XYZ");
t3 = [2 1 1];

circ3 = @(u) repmat(t3',1,size(u,2)) + R3 * padarray([cos(u*(-pi) - pi/2)' sin(u*(-pi) - pi/2)'], [0 1], 0, 'post')';

% plot them all
ax = axes();
view(ax, 3);
grid on;
hold on;
axis equal;

xVersor = [0.2 0   0  ];
yVersor = [0   0.2 0  ];
zVersor = [0   0   0.2];

% plot via points and circumferences centers
scatter3(vias(:,1),vias(:,2),vias(:,3),LineWidth=3,Marker='*');
scatter3(circCenters(:,1),circCenters(:,2),circCenters(:,3),LineWidth=1,Marker='x');

% plot circular paths ref systems
quiver3(zeros(1,3), zeros(1,3), zeros(1,3), xVersor, yVersor, zVersor);
quiver3(repmat(t2(1), 1, 3), repmat(t2(2), 1, 3), repmat(t2(3), 1, 3), xVersor*R2, yVersor*R2, zVersor*R2);
quiver3(repmat(t3(1), 1, 3), repmat(t3(2), 1, 3), repmat(t3(3), 1, 3), xVersor*R3, yVersor*R3, zVersor*R3);

% plot linear paths
fplot3(@(u) [1 0 0]*linear1(u),@(u) [0 1 0]*linear1(u),@(u) [0 0 1]*linear1(u), [0 1], LineWidth=2, Color=rand(1,3), DisplayName='Linear path');
fplot3(@(u) [1 0 0]*linear4(u),@(u) [0 1 0]*linear4(u),@(u) [0 0 1]*linear4(u), [0 1], LineWidth=2, Color=rand(1,3), DisplayName='Linear path');

% plot circular paths
fplot3(@(u) [1 0 0]*circ2(u),@(u) [0 1 0]*circ2(u),@(u) [0 0 1]*circ2(u), [0 1], LineWidth=2, Color=rand(1,3), DisplayName='Circular path');
fplot3(@(u) [1 0 0]*circ3(u),@(u) [0 1 0]*circ3(u),@(u) [0 0 1]*circ3(u), [0 1], LineWidth=2, Color=rand(1,3), DisplayName='Circular path');

% plot paths velocities
uInstants = 0:0.1:1;

linear1Val = linear1(uInstants);
dlinear1Val = gradient(linear1Val);
ddlinear1Val = gradient(dlinear1Val);

circ2Val = circ2(uInstants);
dcirc2Val = gradient(circ2Val);
ddcirc2Val = gradient(dcirc2Val);

circ3Val = circ3(uInstants);
dcirc3Val = gradient(circ3Val);
ddcirc3Val = gradient(dcirc3Val);

linear4Val = linear4(uInstants);
dlinear4Val = gradient(linear4Val);
ddlinear4Val = gradient(dlinear4Val);

vals = [linear1Val, circ2Val, circ3Val, linear4Val];
dvals = [dlinear1Val, dcirc2Val, dcirc3Val, dlinear4Val];
ddvals = [ddcirc2Val ddcirc3Val];

normdVals = dvals ./ sqrt(sum(dvals.^2,1));
normddVals = ddvals ./ sqrt(sum(ddvals.^2,1));

quiver3([1 0 0]*vals,[0 1 0]*vals,[0 0 1]*vals, [1 0 0]*normdVals,[0 1 0]*normdVals,[0 0 1]*normdVals,0, DisplayName='Velocity');
quiver3([1 0 0]*[circ2Val circ3Val],[0 1 0]*[circ2Val circ3Val],[0 0 1]*[circ2Val circ3Val], [1 0 0]*normddVals,[0 1 0]*normddVals,[0 0 1]*normddVals,0, DisplayName='Acceleration');

legend('','','','','', 'Linear path', 'Linear path', 'Circular path', 'Circular path', 'Velocity', 'Acceleration');

text(vias(:,1),vias(:,2)+0.1,vias(:,3)+0.1,cellstr(num2str([1;2;3;4;5]))', "FontWeight","bold", 'FontSize', 15);

% saveas(gcf, 'homework_5/3d_trajectory.png');


%% 3D trajectory with polynomial function

instants = [0 1 2 3 4]; 

A_x = sym('a',[1 size(vias,1)]);
A_y = sym('a',[1 size(vias,1)]);
A_z = sym('a',[1 size(vias,1)]);

% Vandermonde matrix
V = zeros(3, size(vias,1),size(vias,1));

for ax = [1,2,3]
    for i = 1:size(vias,1)
        for j = 1:size(vias,1)
            V(ax, i, j) = instants(i)^(j-1);
        end    
    end
end

a_x = inv(squeeze(V(1,:,:)))*vias(:,1);
a_y = inv(squeeze(V(2,:,:)))*vias(:,2);
a_z = inv(squeeze(V(3,:,:)))*vias(:,3);

traj_x = @(t) (repelem(t, size(A_x, 2)).^((1:size(A_x, 2))-1) * a_x);
traj_y = @(t) (repelem(t, size(A_y, 2)).^((1:size(A_y, 2))-1) * a_y);
traj_z = @(t) (repelem(t, size(A_z, 2)).^((1:size(A_z, 2))-1) * a_z);

figure(2);
view(ax, 3);
grid on;
hold on;
axis equal;

scatter3(vias(:,1),vias(:,2),vias(:,3),LineWidth=3,Marker='*', Color='b', DisplayName='Via points');
fplot3(traj_x,traj_y,traj_z, [instants(1) instants(end)], LineWidth=2, Color='r', DisplayName='Polynomial path');
legend();

text(vias(:,1),vias(:,2)+0.1,vias(:,3)+0.1,cellstr(num2str([1;2;3;4;5]))', "FontWeight","bold", 'FontSize', 15);

% saveas(gcf, 'homework_5/3d_poly.png');
