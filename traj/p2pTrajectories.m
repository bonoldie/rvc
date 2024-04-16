clear all;
clc;

syms t ti tm tf;
syms qi dqi ddqi dddqi qm qf dqf ddqf dddqf;
syms a0 a1 a2 a3  a4 a5 a6 a7 a8;

%% linear trajector
q(t) = a0 + (a1*(t-ti));

solLinear = solve([ q(ti) == qi, q(tf) == qf ], [a0, a1]);
qLinear = subs(q, [a0, a1], [solLinear.a0, solLinear.a1]);

qFinal = subs(qLinear, [qi,qf, ti, tf], [0, pi, 0, 1]);
dqFinal = gradient(qFinal);

fplot(qFinal, [0,1]);
fplot(dqFinal, [0,1]);

% linear trajectory
q(t) = a0 + (a1*(t-ti));

solLinear = solve([ q(ti) == qi, q(tf) == qf ], [a0, a1]);
qLinear = subs(q, [a0, a1], [solLinear.a0, solLinear.a1]);

qFinal = subs(qLinear, [qi,qf, ti, tf], [0, pi, 0, 1]);
dqFinal = gradient(qFinal);

figure(1);
fplot(qFinal, [0,1]);
title("q(t)");
xlabel("t");
ylabel("q");

figure(2);
fplot(dqFinal, [0,1]);
title("dq(t)");
xlabel("t");
ylabel("q");

%% parabolic trajectory
q_a(t) = a0 + (a1*(t-ti)) + (a2*(t-ti)^2);
q_d(t) = a3 + (a4*(t-tm)) + (a5*(t-tm)^2);

solAccelParabolic = solve([ q_a(ti) == qi, q_a(tm) == qm, subs(gradient(q_a), t, ti) == dqi ], [a0, a1, a2]);
qAccelParabolic = subs(q_a, [a0, a1, a2], [solAccelParabolic.a0, solAccelParabolic.a1, solAccelParabolic.a2]);

solDecParabolic = solve([ q_d(tf) == qf, q_d(tm) == qm, subs(gradient(q_d), t, tf) == dqf ], [a3, a4, a5]);
qDecParabolic = subs(q_d, [a3, a4, a5], [solDecParabolic.a3, solDecParabolic.a4, solDecParabolic.a5]);

qAccelFinal = subs(qAccelParabolic, [qi, dqi, qm, ti, tm], [0, 0, pi/2, 0, 0.5]);
dqAccelFinal = gradient(qAccelFinal);
ddqAccelFinal = gradient(dqAccelFinal);

qDecFinal = subs(qDecParabolic, [qm, qf, dqf, tm, tf], [pi/2, pi, 0, 0.5, 1]);
dqDecFinal = gradient(qDecFinal);
ddqDecFinal = gradient(dqDecFinal);

figure(1);
hold on;
fplot(qAccelFinal, [0,0.5]);
fplot(qDecFinal, [0.5,1]);
title("q_a(t)");
xlabel("t");
ylabel("q");

figure(2);
hold on;
fplot(dqAccelFinal, [0,0.5]);
fplot(dqDecFinal, [0.5,1]);
title("dq_a(t)");
xlabel("t");
ylabel("q");

figure(3);
hold on;
fplot(ddqAccelFinal, [0,0.5]);
fplot(ddqDecFinal, [0.5,1]);
title("ddq_a(t)");
xlabel("t");
ylabel("q");

%% cubic trajectory

q(t) = a0 + (a1*(t-ti)) + (a2*(t-ti)^2) + (a3*(t-ti)^3);

solCubic = solve([ q(ti) == qi, q(tf) == qf, subs(gradient(q),t, ti) == dqi, subs(gradient(q),t, tf) == dqf ], [a0, a1, a2, a3]);
qCubic = subs(q, [a0, a1, a2, a3], [solCubic.a0, solCubic.a1, solCubic.a2, solCubic.a3 ]);

qFinal = subs(qCubic, [qi,dqi,qf, dqf, ti, tf], [0,0, pi, 0, 0, 1]);
dqFinal = gradient(qFinal);
ddqFinal = gradient(dqFinal);
dddqFinal = gradient(ddqFinal);

figure(1);
hold on;
fplot(qFinal, [0, 1]);
title("q_a(t)");
xlabel("t");
ylabel("q");

figure(2);
hold on;
fplot(dqFinal, [0, 1]);
title("dq_a(t)");
xlabel("t");
ylabel("q");

figure(3);
hold on;
fplot(ddqFinal, [0, 1]);
title("ddq_a(t)");
xlabel("t");
ylabel("q");

figure(4);
hold on;
fplot(dddqFinal, [0, 1]);
title("dddq_a(t)");
xlabel("t");
ylabel("q");


%% 5th-order trajectory

q(t) = a0 + (a1*(t-ti)) + (a2*(t-ti)^2) + (a3*(t-ti)^3) + (a4*(t-ti)^4)+ (a5*(t-ti)^5);

sol5thOrder = solve([ q(ti) == qi, q(tf) == qf, subs(gradient(q),t, ti) == dqi,subs(gradient(gradient(q)),t, ti) == ddqi, subs(gradient(q),t, tf) == dqf , subs(gradient(gradient(q)),t, tf) == ddqf,], [a0, a1, a2, a3, a4, a5]);
q5thOrder = subs(q, [a0, a1, a2, a3, a4, a5], [sol5thOrder.a0, sol5thOrder.a1, sol5thOrder.a2, sol5thOrder.a3, sol5thOrder.a4, sol5thOrder.a5 ]);

qFinal = subs(q5thOrder, [qi,dqi, ddqi, qf, dqf, ddqf, ti, tf], [0, 0,0 pi, 0, 0, 0, 1]);
dqFinal = gradient(qFinal);
ddqFinal = gradient(dqFinal);
dddqFinal = gradient(ddqFinal);

figure(1);
hold on;
fplot(qFinal, [0, 1]);
title("q_a(t)");
xlabel("t");
ylabel("q");

figure(2);
hold on;
fplot(dqFinal, [0, 1]);
title("dq_a(t)");
xlabel("t");
ylabel("q");

figure(3);
hold on;
fplot(ddqFinal, [0, 1]);
title("ddq_a(t)");
xlabel("t");
ylabel("q");

figure(4);
hold on;
fplot(dddqFinal, [0, 1]);
title("dddq_a(t)");
xlabel("t");
ylabel("q");

%% 7th-order trajectory

q(t) = a0 + (a1*(t-ti)) + (a2*(t-ti)^2) + (a3*(t-ti)^3) + (a4*(t-ti)^4) + (a5*(t-ti)^5) + (a6*(t-ti)^6) + (a7*(t-ti)^7);

sol7thOrder = solve([ q(ti) == qi, q(tf) == qf, subs(gradient(q),t, ti) == dqi,subs(gradient(gradient(q)),t, ti) == ddqi,subs(gradient(q),t, ti) == dqi, subs(gradient(gradient(gradient(q))),t, ti) == dddqi, subs(gradient(q),t, tf) == dqf , subs(gradient(gradient(q)),t, tf) == ddqf, subs(gradient(gradient(gradient(q))),t, tf) == dddqf], [a0, a1, a2, a3, a4, a5, a6, a7]);
q7thOrder = subs(q, [a0, a1, a2, a3, a4, a5, a6, a7], [sol7thOrder.a0, sol7thOrder.a1, sol7thOrder.a2, sol7thOrder.a3, sol7thOrder.a4, sol7thOrder.a5, sol7thOrder.a6, sol7thOrder.a7]);

qFinal = subs(q7thOrder, [qi,dqi, ddqi, dddqi, qf, dqf, ddqf, dddqf, ti, tf], [0, 0, 0, 0, pi, 0, 0, 0, 0, 1]);
dqFinal = gradient(qFinal);
ddqFinal = gradient(dqFinal);
dddqFinal = gradient(ddqFinal);
ddddqFinal = gradient(dddqFinal);

figure(1);
hold on;
fplot(qFinal, [0, 1]);
title("q_a(t)");
xlabel("t");
ylabel("q");

figure(2);
hold on;
fplot(dqFinal, [0, 1]);
title("dq_a(t)");
xlabel("t");
ylabel("q");

figure(3);
hold on;
fplot(ddqFinal, [0, 1]);
title("ddq_a(t)");
xlabel("t");
ylabel("q");

figure(4);
hold on;
fplot(dddqFinal, [0, 1]);
title("dddq_a(t)");
xlabel("t");
ylabel("q");

figure(5);
hold on;
fplot(ddddqFinal, [0, 1]);
title("ddddq_a(t)");
xlabel("t");
ylabel("q");

