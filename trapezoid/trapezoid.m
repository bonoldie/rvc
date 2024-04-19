close all;
clear all;
clc;

syms t ti tf ta td;
syms qi dqi qc dqc qf dqf;

qAcc(t) = qi + dqi*(t-ti) + ((dqc - dqi)*(t-ti)^2)/(2*ta);
qCons(t) = qi + dqi*(ta/2) + dqc*(t-ti-(ta/2));
qDec(t) = qf - dqf*(tf - t) - ((dqc - dqf)*((tf - t)^2))/(2*td);

tuningAccSyms = [ti ta qi dqi qc dqc];
tuningAcc =     [0  3  0  0   5  5];

tuningConstSyms = tuningAccSyms;
tuningConst = tuningAcc;

tuningDecSyms = [td tf qf dqf dqc];
tuningDec =     [3  10 40 0   5];

qAccEval = subs(qAcc, tuningAccSyms, tuningAcc);
qConstEval = subs(qCons, tuningConstSyms, tuningConst);
qDecEval = subs(qDec, tuningDecSyms, tuningDec);

figure(1);
hold on;

subplot(1,2,1);
hold on;
title('position');
fplot(qAccEval, [0 3]);
fplot(qConstEval, [3 7]);
fplot(qDecEval, [7 10]);
legend({'acc. phase','const. phase','dec. phase'},'Location','southeast');

subplot(1,2,2);
hold on;
title('velocity');
fplot(gradient(qAccEval, t), [0 3]);
fplot(gradient(qConstEval, t), [3 7]);
fplot(gradient(qDecEval, t), [7 10]);
legend({'acc. phase','const. phase','dec. phase'},'Location','southeast');

figure(2);
hold on;
title('acceleration');
fplot(gradient(gradient(qAccEval, t),t), [0 3]);
fplot(gradient(gradient(qConstEval, t),t), [3 7]);
fplot(gradient(gradient(qDecEval, t),t), [7 10]);
legend({'acc. phase','const. phase','dec. phase'},'Location','southeast');

