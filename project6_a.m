clear all;
close all;
clc;

% shortest path on sphere
syms x y z real;
syms firstPlaneMeshFun(x,y);

% sphere center and radius
p0 = [5 0 0]';
r = 5;

theta = rand(3,1)*pi;
phi = rand(3,1)*2*pi;

% sphere surface points 
p1 = p0+r*[sin(theta(1))*cos(phi(1)); sin(theta(1))*sin(phi(1)); cos(theta(1))];
p2 = p0+r*[sin(theta(2))*cos(phi(2)); sin(theta(2))*sin(phi(2)); cos(theta(2))];
p3 = p0+r*[sin(theta(3))*cos(phi(3)); sin(theta(3))*sin(phi(3)); cos(theta(3))];

sphereEq = (x-p0(1))^2 +(y-p0(2))^2 +(z-p0(3))^2  == r^2;

firstPlaneEq = det([
    [x y z] 1
    [p0 p1 p2]' [1;1;1]
]) == 0;

secondPlaneEq = det([
    [x y z] 1
    [p0 p2 p3]' [1;1;1]
]) == 0;

p1_2 = p2 - p1;
p1_2linspace = p1 + p1_2*linspace(0,1,10);
p1_2linspace_centered = p1_2linspace - p0;
p1_2linspace_projected = (p1_2linspace_centered ./ sqrt(sum(p1_2linspace_centered.^2,1))) * r;
p1_2linspace_projected = p1_2linspace_projected  + p0;

p2_3 = p3 - p2;
p2_3linspace = p2 + p2_3*linspace(0,1,100);
p2_3linspace_centered = p2_3linspace - p0;
p2_3linspace_projected = (p2_3linspace_centered ./ sqrt(sum(p2_3linspace_centered.^2,1))) * r;
p2_3linspace_projected = p2_3linspace_projected  + p0;


[az1,el1] = cart2sph([1 0 0]*(p1-p0),[0 1 0]*(p1-p0),[0 0 1]*(p1-p0));
[az2,el2] = cart2sph([1 0 0]*(p2-p0),[0 1 0]*(p2-p0),[0 0 1]*(p2-p0));
[az3,el3] = cart2sph([1 0 0]*(p3-p0),[0 1 0]*(p3-p0),[0 0 1]*(p3-p0));

traj1_2_sph = @(u) repmat([az1,el1]',1,size(u, 2)) + ([az2,el2] - [az1,el1])'*u;
traj2_3_sph = @(u) repmat([az2,el2]',1,size(u, 2)) + ([az3,el3] - [az2,el2])'*u;

% plot them all
ax = axes();
view(ax, 3);
hold on;
grid on;

[xsph, yshp, zsph] = sphere;
xsph = r * xsph + p0(1);
yshp = r * yshp + p0(2);
zsph = r * zsph + p0(3);

sphereS = surf(xsph,yshp,zsph, FaceAlpha=0.2, EdgeAlpha=0.2);

[X,Y] = meshgrid(-10:1:10,-10:1:10);
[Z] = meshgrid(-10:1:10);

firstPlaneMeshFun(x,y) = solve(firstPlaneEq, z);
firstPlaneS = surf(X, Y,double(firstPlaneMeshFun(X, Y)), FaceAlpha=0.2, EdgeAlpha=0.2);

secondPlaneMeshFun(x,y) = solve(secondPlaneEq, z);
secondPlaneS = surf(X, Y,double(secondPlaneMeshFun(X, Y)), FaceAlpha=0.2, EdgeAlpha=0.2);

% via points
scatter3([1 0 0]*[p1 p2 p3],[0 1 0]*[p1 p2 p3],[0 0 1]*[p1 p2 p3], 'r', Marker='*', LineWidth=2);

plot3([1 0 0]*p1_2linspace,[0 1 0]*p1_2linspace,[0 0 1]*p1_2linspace, 'b',LineStyle='--', LineWidth=1);
plot3([1 0 0]*p2_3linspace,[0 1 0]*p2_3linspace,[0 0 1]*p2_3linspace, 'b',LineStyle='--', LineWidth=1);


% trajectories plots
u = 0:0.01:1;

[traj1_2X, traj1_2Y, traj1_2Z] = sph2cart([1 0]*traj1_2_sph(u), [0 1]*traj1_2_sph(u), r*ones(1,size(u,2)));
plot3(traj1_2X + p0(1), traj1_2Y + p0(2), traj1_2Z + p0(3), 'g', LineWidth=2);

[traj2_3X, traj2_3Y, traj2_3Z] = sph2cart([1 0]*traj2_3_sph(u), [0 1]*traj2_3_sph(u), r*ones(1,size(u,2)));
plot3(traj2_3X + p0(1), traj2_3Y + p0(2), traj2_3Z + p0(3), 'g', LineWidth=2);

% plot3([1 0 0]*p1_2linspace_projected,[0 1 0]*p1_2linspace_projected,[0 0 1]*p1_2linspace_projected, 'g', LineWidth=2);
% plot3([1 0 0]*p2_3linspace_projected,[0 1 0]*p2_3linspace_projected,[0 0 1]*p2_3linspace_projected, 'g', LineWidth=2);

axis equal;

xlim([-10 10]);
zlim([-10 10]);
ylim([-10 10]);

view([-10 -10 10]);

figure(2);
hold on;
subplot(121)
plot(u,[1 0]*traj2_3_sph(u), 'g', LineWidth=1);

subplot(122)
plot(u,[0 1]*traj2_3_sph(u), 'b', LineWidth=1);
