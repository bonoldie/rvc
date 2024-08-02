clear all;
close all;
clc;

set(0, 'DefaultAxesFontSize', 13);
set(0, 'DefaultFigurePosition', [0,0,1080,720]);


% Smoothing cubic splines

syms t spline_k(t) dd_spline_k(t) real;

T_vals = [1 1 1 1 1 1 1 1];
q = [0 3 5 10 12 0 -2 -10 -5];

instants = cumsum([0 T_vals]);

figure(1);
hold on;
MUs = 0.2:0.2:1;


matlab_splines_coeffs = {};

for i=MUs 
    [s, dds] = SmoothSpline(T_vals, q, i);
    timeInterval = instants(1):0.1:instants(end);
    
    resultSpline = spline(instants, s, timeInterval);
    matlab_splines_coeffs{end+1} = spline(instants, s).coefs;

    subplot(121);
    hold on;
    plot(timeInterval, resultSpline, DisplayName=strjoin({'mu: ' num2str(i)}));

    subplot(122);
    hold on;
    plot(timeInterval, gradient(gradient(resultSpline)), DisplayName=strjoin({'mu: ' num2str(i)}));

end

subplot(121);
scatter(instants,q,'o', 'DisplayName', 'Via points', 'LineWidth', 2);
legend;
xlabel('t');
ylabel('q', 'Rotation',0);

subplot(122);
legend;
xlabel('t');
ylabel('$$\ddot{q}$$','Interpreter','latex', 'Rotation',0);

sgtitle('Smoothing splines');

% saveas(gcf,'homework_3_4/smoothing_splines.png');

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

W = diag([0.01 ones([1 N-1]) 0.01]); % diag(1 - normpdf(N/2- (0:N))).*2;

A = double(subs(A,T,T_vals'));
C = double(subs(C,T,T_vals'));

dds = inv(A + lambda*C*inv(W)*C')*C*q';
s = inv(W + lambda * C' * inv(A) * C) * W * q';

end