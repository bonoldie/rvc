close all;
clear;
clc;

% we have to build a trapezoidal velocity trajectory to match a defined motion
% these are the parameters needed to plan the trajectory

% initial and final position
qi_motion = 0;
qf_motion = 20;

% initial and final time
ti = 0;
tf = 4;

% acceleration phase duration
t_acc = 1;

% deceleration phase duration
t_dec = 1;

% symbols for trajectory equations
syms ac0 ac1 ac2 co0 co1 de0 de1 de2 t;
assume([ac0 ac1 ac2 co0 co1 de0 de1 de2], 'real');
syms t1 t2;

q_acc(t) = ac0 + ac1*t + ac2*t^2;
dq_acc = gradient(q_acc, t);
ddq_acc = gradient(dq_acc, t);

q_const(t) = co0 + co1*t;
dq_const = gradient(q_const, t);

q_dec(t) = de0 + de1*t + de2*t^2;
dq_dec = gradient(q_dec, t);
ddq_dec = gradient(dq_dec, t);

% acceleration phase constraints
accPlanning = [
    q_acc(ti) == qi_motion ...
    dq_acc(ti) == 0 ...
    t1 - ti == t_acc
];

% constant phase constraints
constPlanning = [
    q_const(t1) == q_acc(t1) ...
    dq_const(t1) == dq_acc(t1) ...
    dq_const(t2) == dq_dec(t2) ...
    q_const(t2) == q_dec(t2)
];
 
% deceleration phase constraints
decPlanning = [
    q_dec(tf) == qf_motion ...
    dq_dec(tf) == 0 ...
    tf - t2 == t_dec
];

planningSolution = solve([accPlanning constPlanning decPlanning]);

figure(1);
subplot(3,1,1);
hold on;
fplot(subs(q_acc, [ac0 ac1 ac2], [planningSolution.ac0 planningSolution.ac1 planningSolution.ac2]),[ti double(planningSolution.t1)]);
fplot(subs(q_const, [co0 co1], [planningSolution.co0 planningSolution.co1]),[double(planningSolution.t1) double(planningSolution.t2)]);
fplot(subs(q_dec, [de0 de1 de2],[planningSolution.de0 planningSolution.de1 planningSolution.de2]),[double(planningSolution.t2) tf]);
title('Position');

subplot(3,1,2);
hold on;
fplot(subs(dq_acc, [ac0 ac1 ac2], [planningSolution.ac0 planningSolution.ac1 planningSolution.ac2]),[ti double(planningSolution.t1)]);
fplot(subs(dq_const, [co0 co1], [planningSolution.co0 planningSolution.co1]),[double(planningSolution.t1) double(planningSolution.t2)]);
fplot(subs(dq_dec, [de0 de1 de2],[planningSolution.de0 planningSolution.de1 planningSolution.de2]),[double(planningSolution.t2) tf]);
title('Velocity');