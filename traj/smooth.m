clear all;
close all;
clc;

T_vals = [1 2 3 2 1 3 2 1];
q = [0 3 5 10 12 0 -2 -10 -5];

instants = cumsum([0 T_vals]);

figure(1);
hold on;
scatter(instants,q,'o', DisplayName='Via points');

MUs = 0.2:0.2:1;

for i=MUs 
    s = SmoothSpline(T_vals, q, i);
    timeInterval = instants(1):0.1:instants(end);
   
    resultSpline = spline(instants, s, timeInterval);
    plot(timeInterval, resultSpline, DisplayName=strjoin({'mu: ' num2str(i)}));
end

legend;
title('Smooth splines')

function s = SmoothSpline(T_vals, q, mu)

% Number of segments
N = size(T_vals,2);

lambda = ((1-mu)/(6*mu));

T = sym('T', [N 1]);

A = sym(zeros(N+1, N+1));
C = sym(zeros(N+1, N+1));

A = A + diag(T, -1);
A = A + 2*diag([T; 0]+ [0; T]);
A = A + diag(T, 1);

C = C + diag(6./T, -1);
C = C + diag(-6./([T; 0]+ [0; T]));
C = C + diag(6./T, 1);

W = eye(N + 1);

A = double(subs(A,T,T_vals'));
C = double(subs(C,T,T_vals'));

s = inv(W + lambda * C' * inv(A) * C) * W * q';

end