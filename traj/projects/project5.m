close all;
clear all;
clc;

% 3D trajectory
syms s;

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
fplot3(@(u) [1 0 0]*linear1(u),@(u) [0 1 0]*linear1(u),@(u) [0 0 1]*linear1(u), [0 1], LineWidth=2, Color=rand(1,3));
fplot3(@(u) [1 0 0]*linear4(u),@(u) [0 1 0]*linear4(u),@(u) [0 0 1]*linear4(u), [0 1], LineWidth=2, Color=rand(1,3));

% plot circular paths
fplot3(@(u) [1 0 0]*circ2(u),@(u) [0 1 0]*circ2(u),@(u) [0 0 1]*circ2(u), [0 1], LineWidth=2, Color=rand(1,3));
fplot3(@(u) [1 0 0]*circ3(u),@(u) [0 1 0]*circ3(u),@(u) [0 0 1]*circ3(u), [0 1], LineWidth=2, Color=rand(1,3));

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

quiver3([1 0 0]*vals,[0 1 0]*vals,[0 0 1]*vals, [1 0 0]*normdVals,[0 1 0]*normdVals,[0 0 1]*normdVals,0);
quiver3([1 0 0]*[circ2Val circ3Val],[0 1 0]*[circ2Val circ3Val],[0 0 1]*[circ2Val circ3Val], [1 0 0]*normddVals,[0 1 0]*normddVals,[0 0 1]*normddVals,0);





