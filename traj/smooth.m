clear all;
close all;
clc;

syms t spline_k(t) dd_spline_k(t);

T_vals = [1 1 1 1 1 1 1 1];
q = [0 3 5 10 12 0 -2 -10 -5];

instants = cumsum([0 T_vals]);

figure(1);
hold on;
scatter(instants,q,'o', DisplayName='Via points');

MUs = 0.2:0.2:1;

for i=MUs 
    [s, dds] = SmoothSpline(T_vals, q, i);
    timeInterval = instants(1):0.1:instants(end);
   
    resultSpline = spline(instants, s, timeInterval);
    plot(timeInterval, resultSpline, DisplayName=strjoin({'mu: ' num2str(i)}));
end

legend;
title('Smooth splines (matlab spline function)')

figure(2);
hold on;
scatter(instants,q,'o', DisplayName='Via points');

MUs = 0.2:0.2:1;

for i=MUs 
    [s, dds] = SmoothSpline(T_vals, q, i);
    splineColor = [rand,rand,rand];

    for k = 1:size(T_vals,2)
        %spline_k(t) = ((s(k+1)/T_vals(k))-((T_vals(k)*dds(k+1))/6)) * (t - instants(k)) + (((s(k)/T_vals(k))-(T_vals(k)*dds(k)/6))) * (instants(k+1) - t) + (dds(k)/(6*T_vals(k))) * (instants(k+1) - t)^3 + (dds(k+1)/(6*T_vals(k)))*(t-instants(k))^3;
        %spline_k(t) = ((dds(k+1) - dds(k))/ (6*T_vals(k)))*(t-instants(k))^3 + (dds(k)/2)*(t-instants(k))^2 + ((s(k+1)-s(k))/T_vals(k) - ((dds(k+1) + 2*dds(k))/6)*T_vals(k)) * (t - instants(k)) + s(k);
        %dd_spline_k(t) = (dds(k)/T_vals(k))*(instants(k+1)-t) + (dds(k+1)/T_vals(k))*(t-instants(k));
        
        fplot(spline_k(t),[instants(k) instants(k+1)], 'Color',splineColor)
    end
end
% legend;
title('Smooth splines')

function [s, dds] = SmoothSpline(T_vals, q, mu)

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

W = diag(1 - normpdf(N/2- (0:N))).*2;

A = double(subs(A,T,T_vals'));
C = double(subs(C,T,T_vals'));

s = inv(W + lambda * C' * inv(A) * C) * W * q';

%dds = inv(A + lambda*C*inv(W)*C')*C*q';
dds = inv(A)*C*s;
end