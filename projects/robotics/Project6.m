clear all;
close all;
clc;

% Trajectory on a sphere

syms x y z;

% sphere center and radius
p0 = [5 0 0]';
r = 5;

% random points
theta = rand(3,1)*pi; % lat 
phi = rand(3,1)*2*pi; % long

p1_O = r*[sin(theta(1))*cos(phi(1)); sin(theta(1))*sin(phi(1)); cos(theta(1))];
p2_O = r*[sin(theta(2))*cos(phi(2)); sin(theta(2))*sin(phi(2)); cos(theta(2))];
p3_O = r*[sin(theta(3))*cos(phi(3)); sin(theta(3))*sin(phi(3)); cos(theta(3))];

p1 = p0 + p1_O;
p2 = p0 + p2_O;
p3 = p0 + p3_O;

p1_2 = p2 - p1;
p1_2linspace = p1 + p1_2*linspace(0,1,10);
p1_2linspace_centered = p1_2linspace - p0;
p1_2linspace_projected = (p1_2linspace_centered ./ sqrt(sum(p1_2linspace_centered.^2,1))) * r  + p0;

p2_3 = p3 - p2;
p2_3linspace = p2 + p2_3*linspace(0,1,100);
p2_3linspace_centered = p2_3linspace - p0;
p2_3linspace_projected = ((p2_3linspace_centered ./ sqrt(sum(p2_3linspace_centered.^2,1))) * r) + p0;

p3_1 = p1 - p3;
p3_1linspace = p3 + p3_1*linspace(0,1,100);
p3_1linspace_centered = p3_1linspace - p0;
p3_1linspace_projected = ((p3_1linspace_centered ./ sqrt(sum(p3_1linspace_centered.^2,1))) * r) + p0;

plane1_2_O = det([
    [x y z] 1
    [p0 p1 p2]' [1;1;1]
]) == 0;


theta1_2 = acos(dot(p1_O/r, p2_O/r)); 
theta2_3 = acos(dot(p2_O/r, p3_O/r)); 
theta3_1 = acos(dot(p3_O/r, p1_O/r)); 

[B, I]  = sort([theta1_2, theta2_3, theta3_1]);

disp(strcat('To minimize the travel distance we must use the segments ', sprintf(' %i and %i', I(1:2))));

% Great circle poles
GCP1_2 = (cross(p1_O/r, p2_O/r)/sin(theta1_2))*r;
GCP2_3 = (cross(p2_O/r, p3_O/r)/sin(theta2_3))*r;
GCP3_1 = (cross(p3_O/r, p1_O/r)/sin(theta3_1))*r;

traj1_2 = @(u) ((cos(u).*(p1_O/r) + sin(u) .* cross(GCP1_2/r, p1_O/r)) * r) + p0;
traj2_3 = @(u) ((cos(u).*(p2_O/r) + sin(u) .* cross(GCP2_3/r, p2_O/r)) * r) + p0;
traj3_1 = @(u) ((cos(u).*(p3_O/r) + sin(u) .* cross(GCP3_1/r, p3_O/r)) * r) + p0;

% plot them all
ax = axes();
view(ax, 3);
hold on;
grid on;

[X,Y] = meshgrid(-10:1:10,-10:1:10);
[Z] = meshgrid(-10:1:10);

[xsph, yshp, zsph] = sphere;
xsph = r * xsph + p0(1);
yshp = r * yshp + p0(2);
zsph = r * zsph + p0(3);

sphereS = surf(xsph,yshp,zsph, FaceAlpha=0.2, EdgeAlpha=0.2);

% via points
scatter3([1 0 0]*[p1 p2 p3],[0 1 0]*[p1 p2 p3],[0 0 1]*[p1 p2 p3], 'r', Marker='*', LineWidth=2);

plot3([1 0 0]*p1_2linspace,[0 1 0]*p1_2linspace,[0 0 1]*p1_2linspace, 'b',LineStyle='--', LineWidth=1);
plot3([1 0 0]*p2_3linspace,[0 1 0]*p2_3linspace,[0 0 1]*p2_3linspace, 'b',LineStyle='--', LineWidth=1);
plot3([1 0 0]*p3_1linspace,[0 1 0]*p3_1linspace,[0 0 1]*p3_1linspace, 'b',LineStyle='--', LineWidth=1);

plane1_2_O_MeshFun(x,y) = solve(plane1_2_O, z);
plane1_2_OS = surf(X, Y,double(plane1_2_O_MeshFun(X, Y)), FaceAlpha=0.2, EdgeAlpha=0.2);

quiver3(p0(1),p0(2),p0(3), p1_O(1),p1_O(2),p1_O(3),'r', LineWidth=1);
quiver3(p0(1),p0(2),p0(3), p2_O(1),p2_O(2),p2_O(3),'r', LineWidth=1);
quiver3(p0(1),p0(2),p0(3), p3_O(1),p3_O(2),p3_O(3),'r', LineWidth=1);

quiver3(p0(1),p0(2),p0(3), GCP1_2(1),GCP1_2(2),GCP1_2(3),'black', LineWidth=1);

% quiver3(p0(1),p0(2),p0(3), p1_2_test(1),p1_2_test(2),p1_2_test(3),'magenta', LineWidth=1);

traj1_2_eval = traj1_2(0:0.1:theta1_2);
traj2_3_eval = traj2_3(0:0.1:theta2_3);
traj3_1_eval = traj3_1(0:0.1:theta3_1);

geodesic_plots(1) = plot3([1 0 0]*traj1_2_eval,[0 1 0]*traj1_2_eval,[0 0 1]*traj1_2_eval, LineStyle="-", LineWidth=2, DisplayName='Geodesic line 1-2');
geodesic_plots(2) = plot3([1 0 0]*traj2_3_eval,[0 1 0]*traj2_3_eval,[0 0 1]*traj2_3_eval, LineStyle="-", LineWidth=2, DisplayName='Geodesic line 2-3');
geodesic_plots(3) = plot3([1 0 0]*traj3_1_eval,[0 1 0]*traj3_1_eval,[0 0 1]*traj3_1_eval, LineStyle="-", LineWidth=2, DisplayName='Geodesic line 3-1');

legend(geodesic_plots, {'Geodesic line 1-2','Geodesic line 2-3','Geodesic line 3-1'});

% view settings
axis equal;
xlim([-10 10]);
zlim([-10 10]);
ylim([-10 10]);
view([-10 -10 10]);
