close all;
clear;
clc;

vias = [
    0, 0;
    1, 1;
    2, 5;
    3, 1;
    4, 0;
    5, 4;
];
    
ti = 0;

disp(array2table(vias, 'VariableNames', {'t', 'q'}))
disp('-------------')

% high order polynomial (find the solution of a problem of type q=V*a)

syms a0 a1 a2 a3 a4 a5 t;

V = zeros(size(vias,1));

for i = 1:size(vias,1)
    for j = 1:size(vias,1)
        V(i, j) = vias(i, 1)^(j-1);
    end    
end

highOrderPolynomialProblem = vias(:,2) == V*[a0;a1;a2;a3;a4;a5];

highOrderPolynomialSol = solve(highOrderPolynomialProblem, [a0 a1 a2 a3 a4 a5]);

q = subs(a0 + a1*t + a2*t^2 + a3*t^3 + a4*t^4 + a5*t^5,[a0 a1 a2 a3 a4 a5], [ highOrderPolynomialSol.a0  highOrderPolynomialSol.a1  highOrderPolynomialSol.a2  highOrderPolynomialSol.a3  highOrderPolynomialSol.a4, highOrderPolynomialSol.a5]);
dq = gradient(q, t);
ddq = gradient(dq, t);



figure(1);
hold on;
fplot(q, [0, 5]);
scatter(vias(:, 1),vias(:, 2));
title('5th order polynomial multipoint trajectory')

figure(2);
hold on;
fplot(dq, [0, 5]);
scatter(vias(:, 1),vias(:, 2));
title('5th order polynomial multipoint trajectory velocity')