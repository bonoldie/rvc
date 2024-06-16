clear all;
close all;
clc;

% shortest path on sphere

syms r theta1 theta2 theta3 phi1 phi2 phi3 real;
assume(theta1 >= 0);
assume(theta1 <= pi);
assume(theta2 >= 0);
assume(theta2 <= pi);
assume(theta3 >= 0);
assume(theta3 <= pi);

assume(phi1 >= 0);
assume(phi1 <= 2*pi);
assume(phi2 >= 0);
assume(phi2 <= 2*pi);
assume(phi3 >= 0);
assume(phi3 <= 2*pi);

% sphere center
p0 = [5 0 0]';

% sphere surface points 
p1 = [2 0 0]';
p2 = [2 4 0]';
p3 = [0 0 5]';

sphereProblem = [
   p1 == p0 + r*[sin(theta1)*cos(phi1); sin(theta1)*sin(phi1);cos(theta1)]
   p2 == p0 + r*[sin(theta2)*cos(phi2); sin(theta2)*sin(phi2);cos(theta2)]
   p3 == p0 + r*[sin(theta3)*cos(phi3); sin(theta3)*sin(phi3);cos(theta3)]
];

s = solve(sphereProblem);

% axis equal;
