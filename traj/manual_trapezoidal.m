close all;
clear all;
clc;

syms a0 a1 a2 b0 b1 c0 c1 c2 t ti tf qi qf
assume([a0 a1 a2 b0 b1 c0 c1 c2], 'real');

%ti=0;
%tf=5;

%qi=5;
%qf=6;

%vmax=2;

%acceleration

q(t)=a0+a1*t+a2*t^2;
q1(t)=diff(q,t);
q2(t)=diff(q1,t);

%constant vel

g(t)=b0+b1*t;
g1(t)=diff(g,t);
g2(t)=diff(g1,t);

%deceleration

f(t)=c0+c1*t+c2*t^2;
f1(t)=diff(f,t);
f2(t)=diff(f1,t);

%change this value to enlarge constant velocity
tc=(tf-ti)/4;   %tc<=(tf-ti)/2
qc=q(ti+tc);

%eqns=[ q(ti)==qi , q1(ti)==0 , q2(tc)==q1(tc)/(2*tc)];

eqns=[q(ti)==qi, f(tf)==qf, q(tc)==g(tc), g(tf-tc)==f(tf-tc), ... 
    q1(ti)==0, f1(tf)==0, q1(tc)==g1(tc), g1(tf-tc)==f1(tf-tc) ];

s=solve(eqns,[a0 a1 a2 b0 b1 c0 c1 c2])


%%
P=subs(q(t),[a0,a1,a2],[s.a0,s.a1,s.a2]);
V=subs(q1(t),[a0,a1,a2],[s.a0,s.a1,s.a2]);
A=subs(q2(t),[a0,a1,a2],[s.a0,s.a1,s.a2]);

P1=subs(g(t),[b0,b1],[s.b0,s.b1]);
V1=subs(g1(t),[b0,b1],[s.b0,s.b1]);
A1=subs(g2(t),[b0,b1],[s.b0,s.b1]);

P2=subs(f(t),[c0,c1,c2],[s.c0,s.c1,s.c2]);
V2=subs(f1(t),[c0,c1,c2],[s.c0,s.c1,s.c2]);
A2=subs(f2(t),[c0,c1,c2],[s.c0,s.c1,s.c2]);


% pretty_equation(P2);
%%
acc_range=linspace(ti,tc,100);
constant_range=linspace(tc,tf-tc,100);
dec_range=linspace(tf-tc,tf,100);

figure(1)
subplot(2,2,1);

plot(acc_range,subs(P,t,acc_range))
title('pos')
grid('on')
grid("minor")
hold on
plot(constant_range,subs(P1,t,constant_range))
plot(dec_range,subs(P2,t,dec_range))
hold off

subplot(2,2,2)
plot(acc_range,subs(V,t,acc_range))
title('vel')
grid('on')
grid("minor")
hold on
plot(constant_range,subs(V1,t,constant_range))
plot(dec_range,subs(V2,t,dec_range))
hold off

subplot(2,2,3)
plot(acc_range,subs(A,t,acc_range))
title('acc')
grid('on')
grid("minor")
hold on
plot(constant_range,subs(A1,t,constant_range))
plot(dec_range,subs(A2,t,dec_range))
hold off

