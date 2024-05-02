close all;
clear;
clc;

syms a0 a1 a2 t;
assume([a0 a1 a2], 'real');

syms ti tf qi qf;

% quadratic trajectory eq
q(t)=a0+a1*t+a2*t^2;
dq = gradient(q,t);
ddq = gradient(dq,t);

% trajectory specs
ta = 1;
td = 1;

q_accPhase=2;
q_constPhase=6;
q_decPhase=2;

% acceleration phase
accProblem = [            ...
   q(ti) == qi            ...
   q(ti+ta) == q_accPhase ...
   dq(ti) == 0            ...
];

accProblem = subs(accProblem, [ti qi],[0 0]);

accSol = solve(accProblem);

q_acc = subs(q, [a0 a1 a2], [accSol.a0 accSol.a1 accSol.a2]);
dq_acc = subs(dq, [a0 a1 a2], [accSol.a0 accSol.a1 accSol.a2]);
ddq_acc = subs(ddq, [a0 a1 a2], [accSol.a0 accSol.a1 accSol.a2]);

disp('Acceleration phase');
disp(array2table([string(q_acc) string(dq_acc) string(ddq_acc)], 'VariableNames',{'q(t)','dq(t)','ddq(t)'}));

% linear trajectory eq
q(t)=a0+a1*t;
dq = gradient(q,t);

% constant phase
constProblem = [                       ...
   q(ti) == q_acc(ta)                  ...
   q(ti+tc) == q_acc(ta)+q_constPhase  ...
   dq(ti) == dq_acc(ta)                ...   
];

constProblem = subs(constProblem, [ti],[ti+ta]);

constSol = solve(constProblem, [a0 a1]);

q_const = subs(q, [a0 a1], [constSol.a0 constSol.a1]);
dq_const = subs(dq, [a0 a1], [constSol.a0 constSol.a1]);

disp('Constant dq phase');
disp(array2table([string(q_const) string(dq_const)], 'VariableNames',{'q(t)','dq(t)'}));

