clear all;
close all;
clc;

% Joint space trajectories
% Point-to-point trajectories

syms t ti tm tf;
syms qi dqi ddqi dddqi qm qf dqf ddqf dddqf;
syms a0 a1 a2 a3 a4 a5 a6 a7 a8;

%% linear trajector

q(t) = a0 + (a1*(t-ti));

solLinear = solve([ q(ti) == qi, q(tf) == qf ], [a0, a1]);
qLinear = subs(q, [a0, a1], [solLinear.a0, solLinear.a1]);

qFinal = subs(qLinear, [qi,qf, ti, tf], [0, pi, 0, 1]);
dqFinal = gradient(qFinal);

disp("Linear trajectory");
tab = struct2table(solLinear);
disp(tab);

figure(1);
subplot(121);
fplot(qFinal, [0,1]);
title("linear trajectory","q(t)");
xlabel("t");

subplot(122);
fplot(dqFinal, [0,1]);
title("linear trajectory","dq(t)");
xlabel("t");

%% parabolic trajectory

q_a(t) = a0 + (a1*(t-ti)) + (a2*(t-ti)^2);
q_d(t) = a0 + (a1*(t-tm)) + (a2*(t-tm)^2);

solAccelParabolic = solve([ q_a(ti) == qi, q_a(tm) == qm, subs(gradient(q_a), t, ti) == dqi ], [a0, a1, a2]);
qAccelParabolic = subs(q_a, [a0, a1, a2], [solAccelParabolic.a0, solAccelParabolic.a1, solAccelParabolic.a2]);

solDecParabolic = solve([ q_d(tf) == qf, q_d(tm) == qm, subs(gradient(q_d), t, tf) == dqf ], [a0, a1, a2]);
qDecParabolic = subs(q_d, [a0, a1, a2], [solDecParabolic.a0, solDecParabolic.a1, solDecParabolic.a2]);

qAccelFinal = subs(qAccelParabolic, [qi, dqi, qm, ti, tm], [0, 0, pi/2, 0, 0.5]);
dqAccelFinal = gradient(qAccelFinal);
ddqAccelFinal = gradient(dqAccelFinal);

qDecFinal = subs(qDecParabolic, [qm, qf, dqf, tm, tf], [pi/2, pi, 0, 0.5, 1]);
dqDecFinal = gradient(qDecFinal);
ddqDecFinal = gradient(dqDecFinal);

disp("Parabolic trajectory (acc)");
tab1 = struct2table(solAccelParabolic);
disp(tab1);

disp("Parabolic trajectory (dec)");
tab2 = struct2table(solDecParabolic);
disp(tab2);

figure(2);
subplot(131);
hold on;
fplot(qAccelFinal, [0,0.5]);
fplot(qDecFinal, [0.5,1]);
title("parabolic trajectory","q(t)");
xlabel("t");
legend("acceleration","deceleration");

subplot(132);
hold on;
fplot(dqAccelFinal, [0,0.5]);
fplot(dqDecFinal, [0.5,1]);
title("parabolic trajectory","dq(t)");
xlabel("t");
legend("acceleration","deceleration");

subplot(133);
hold on;
fplot(ddqAccelFinal, [0,0.5]);
fplot(ddqDecFinal, [0.5,1]);
title("parabolic trajectory","ddq(t)");
xlabel("t");
legend("acceleration","deceleration");

%% cubic trajectory

q(t) = a0 + (a1*(t-ti)) + (a2*(t-ti)^2) + (a3*(t-ti)^3);

solCubic = solve([ q(ti) == qi, q(tf) == qf, subs(gradient(q),t, ti) == dqi, subs(gradient(q),t, tf) == dqf ], [a0, a1, a2, a3]);
qCubic = subs(q, [a0, a1, a2, a3], [solCubic.a0, solCubic.a1, solCubic.a2, solCubic.a3 ]);

qFinal = subs(qCubic, [qi,dqi,qf, dqf, ti, tf], [0,0, pi, 0, 0, 1]);
dqFinal = gradient(qFinal);
ddqFinal = gradient(dqFinal);
dddqFinal = gradient(ddqFinal);

disp("Cubic trajectory");
tab = struct2table(solCubic);
disp(tab);

figure(3);
subplot(221);
hold on;
fplot(qFinal, [0, 1]);
title("cubic trajectory","q(t)");
xlabel("t");

subplot(222);
hold on;
fplot(dqFinal, [0, 1]);
title("cubic trajectory","dq(t)");
xlabel("t");

subplot(223);
hold on;
fplot(ddqFinal, [0, 1]);
title("cubic trajectory","ddq(t)");
xlabel("t");

subplot(224);
hold on;
fplot(dddqFinal, [0, 1]);
title("cubic trajectory","dddq(t)");
xlabel("t");

%% 5th-order trajectory

q(t) = a0 + (a1*(t-ti)) + (a2*(t-ti)^2) + (a3*(t-ti)^3) + (a4*(t-ti)^4)+ (a5*(t-ti)^5);

sol5thOrder = solve([ q(ti) == qi, q(tf) == qf, subs(gradient(q),t, ti) == dqi,subs(gradient(gradient(q)),t, ti) == ddqi, subs(gradient(q),t, tf) == dqf , subs(gradient(gradient(q)),t, tf) == ddqf,], [a0, a1, a2, a3, a4, a5]);
q5thOrder = subs(q, [a0, a1, a2, a3, a4, a5], [sol5thOrder.a0, sol5thOrder.a1, sol5thOrder.a2, sol5thOrder.a3, sol5thOrder.a4, sol5thOrder.a5 ]);

qFinal = subs(q5thOrder, [qi,dqi, ddqi, qf, dqf, ddqf, ti, tf], [0, 0,0 pi, 0, 0, 0, 1]);
dqFinal = gradient(qFinal);
ddqFinal = gradient(dqFinal);
dddqFinal = gradient(ddqFinal);

disp("5th-order trajectory");
tab = struct2table(sol5thOrder);
disp(tab);

figure(4);
subplot(221);
hold on;
fplot(qFinal, [0, 1]);
title("5th-order trajectory", "q(t)");
xlabel("t");

subplot(222);
hold on;
fplot(dqFinal, [0, 1]);
title("5th-order trajectory", "dq(t)");
xlabel("t");

subplot(223);
hold on;
fplot(ddqFinal, [0, 1]);
title("5th-order trajectory", "ddq(t)");
xlabel("t");

subplot(224);
hold on;
fplot(dddqFinal, [0, 1]);
title("5th-order trajectory", "dddq(t)");
xlabel("t");


%% 7th-order trajectory

q(t) = a0 + (a1*(t-ti)) + (a2*(t-ti)^2) + (a3*(t-ti)^3) + (a4*(t-ti)^4) + (a5*(t-ti)^5) + (a6*(t-ti)^6) + (a7*(t-ti)^7);

sol7thOrder = solve([ q(ti) == qi, q(tf) == qf, subs(gradient(q),t, ti) == dqi,subs(gradient(gradient(q)),t, ti) == ddqi,subs(gradient(q),t, ti) == dqi, subs(gradient(gradient(gradient(q))),t, ti) == dddqi, subs(gradient(q),t, tf) == dqf , subs(gradient(gradient(q)),t, tf) == ddqf, subs(gradient(gradient(gradient(q))),t, tf) == dddqf], [a0, a1, a2, a3, a4, a5, a6, a7]);
q7thOrder = subs(q, [a0, a1, a2, a3, a4, a5, a6, a7], [sol7thOrder.a0, sol7thOrder.a1, sol7thOrder.a2, sol7thOrder.a3, sol7thOrder.a4, sol7thOrder.a5, sol7thOrder.a6, sol7thOrder.a7]);

qFinal = subs(q7thOrder, [qi,dqi, ddqi, dddqi, qf, dqf, ddqf, dddqf, ti, tf], [0, 0, 0, 0, pi, 0, 0, 0, 0, 1]);
dqFinal = gradient(qFinal);
ddqFinal = gradient(dqFinal);
dddqFinal = gradient(ddqFinal);

disp("7th-order trajectory");
tab = struct2table(sol7thOrder);
disp(tab);

figure(5);
subplot(221);
hold on;
fplot(qFinal, [0, 1]);
title("7th-order trajectory","q(t)");
xlabel("t");

subplot(222);
hold on;
fplot(dqFinal, [0, 1]);
title("7th-order trajectory","dq(t)");
xlabel("t");

subplot(223);
hold on;
fplot(ddqFinal, [0, 1]);
title("7th-order trajectory","ddq(t)");
xlabel("t");

subplot(224);
hold on;
fplot(dddqFinal, [0, 1]);
title("7th-order trajectory","dddq(t)");
xlabel("t");



