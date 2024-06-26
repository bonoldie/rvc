close all;
clear;
clc;

%   t  q
vias = [
    0, 0;
    1, 1;
    3, 5;
    5, 1;
    9, 0;
    10, 4;
];

disp(array2table(vias, 'VariableNames', {'t', 'q'}))
disp('-------------')

%% high order polynomial (find the solution of a problem of type q=V*a)

syms t;

A = sym('a',[1 size(vias,1)]);

% Vandermonde matrix
V = zeros(size(vias,1));

for i = 1:size(vias,1)
    for j = 1:size(vias,1)
        V(i, j) = vias(i, 1)^(j-1);
    end    
end

% Instead of setup the problem and the run the 'solve' matlab function and
% can be solved by matrix inversion: A=V^-1*q
a = inv(V)*vias(:,2);

highOrderPolynomialProblem = vias(:,2) == V*A';
highOrderPolynomialSol = solve(highOrderPolynomialProblem, A);

comp = [
    struct2array(highOrderPolynomialSol);
    a';
];

disp(array2table(double(comp)', 'VariableNames', {'a(solve)', 'a(matrix inversion)'}))
disp('-------------')

% q = subs(A,A, [ highOrderPolynomialSol.a0  highOrderPolynomialSol.a1  highOrderPolynomialSol.a2  highOrderPolynomialSol.a3  highOrderPolynomialSol.a4, highOrderPolynomialSol.a5]);
q = (struct2array(highOrderPolynomialSol) * repelem(t, size(A, 2)).^((1:size(A, 2))-1)')';
dq = gradient(q, t);
ddq = gradient(dq, t);

figureTitle = [num2str(size(vias,1)-1) 'th-order multipoint trajectory'];

figure(1);
subplot(131);
hold on;
fplot(q, [vias([1 end],1)]');
scatter(vias(:, 1),vias(:, 2));
title(figureTitle, 'q(t)');

subplot(132);
hold on;
fplot(dq, [vias([1 end],1)]');
title(figureTitle, 'dq(t)');

subplot(133);
hold on;
fplot(ddq, [vias([1 end],1)]');
title(figureTitle, 'ddq(t)');

%% interpolating (cubic) polynomials - imposed velocity at path/initial velocity at via points

% smart velocity constraints
for i=2:size(vias, 1)-1
    vk = (vias(i,2) - vias(i-1,2))/(vias(i,1) - vias(i-1,1));
    vk_plus_1 = (vias(i+1,2) - vias(i,2))/(vias(i+1,1) - vias(i,1));
    
    if(sign(vk_plus_1) * sign(vk) > 0)
        vias(i,3) = (vk + vk_plus_1) / 2;
    else
        vias(i,3) = 0;
    end
    
end

syms t 
A = sym('a',[size(vias, 1)-1 4]);

multipointProblem = {};

% continuity constraints
for i=1:size(vias, 1)-1
    spline_k = A(i, :) * [1 t t^2 t^3].';

    if(i < size(vias, 1)-1)
        spline_k_plus_1 = A(i+1, :) * [1 t t^2 t^3].';
    end
    
    multipointProblem{ end + 1 } =  subs(spline_k, t, vias(i, 1)) == vias(i, 2);
    multipointProblem{ end + 1 } =  subs(spline_k, t, vias(i+1, 1)) == vias(i+1, 2);

    if(i < size(vias, 1)-1)
        multipointProblem{ end + 1 } =  subs(gradient(spline_k_plus_1, t), t, vias(i+1, 1)) == vias(i+1, 3);
        multipointProblem{ end + 1 } =  subs(gradient(spline_k, t), t, vias(i+1, 1)) == vias(i+1, 3);
    end
end

% initial/final velocity constraints
multipointProblem{ end + 1 } = subs(gradient(A(1, :) * [1 t t^2 t^3].', t), t, vias(1, 1)) == 0;
multipointProblem{ end + 1 } = subs(gradient(A(end, :) * [1 t t^2 t^3].', t), t, vias(end, 1)) == 0;

multipointSol = solve([multipointProblem{:}]);

splines = reshape(struct2array(multipointSol), 4, size(vias, 1)-1).' * [1;t;t^2;t^3];

figure(2);
subplot(131);
hold on;
scatter(vias(:, 1), vias(:, 2));
for i=1:size(vias, 1)-1
    fplot(splines(i), [vias(i,1) vias(i+1,1)]);
end
title('Multipoint splines (euler approximation)', 'q(t)')

subplot(132);
hold on;
for i=1:size(vias, 1)-1
    fplot(gradient(splines(i),t), [vias(i,1) vias(i+1,1)]);
end
title('Multipoint splines (euler approximation)', 'dq(t)');

subplot(133);
hold on;
for i=1:size(vias, 1)-1
    fplot(gradient(gradient(splines(i),t)), [vias(i,1) vias(i+1,1)]);
end
title('Multipoint splines (euler approximation)', 'ddq(t)');

%% interpolating (cubic) polynomials - continuous acceleration
dqi = 2;
dqf = 1;

T = [vias(:,1);0] - [0;vias(:,1)];
T = T(2:end-1);

% Setup the tridiagonal A matrix
B = zeros(size(vias,1)-2) + diag(T(3:end),-1); 
B = B + diag(T(1:end-2),1); 
B = B + diag(2*(T(1:end-1) + T(2:end))); 

c = zeros(size(vias,1)-2, 1);

for i=1:size(c,1)
    c(i) = 3*((T(i+1)/T(i))*(vias(i+1, 2) - vias(i, 2)) + (T(i) / T(i+1))*(vias(i+2, 2)-vias(i+1, 2)));

    if i == 1
        c(i) = c(i) - T(i+1) * dqi;
    end

    if i == size(c,1)
        c(i) = c(i) - T(end-1) * dqf;
    end
end

calculated_dq = inv(B) * c;

vias(:,3) = [dqi;calculated_dq;dqf];

A = sym('a',[size(vias, 1)-1 4]);

multipointProblem = {};

% continuity constraints
for i=1:size(vias, 1)-1
    spline_k = A(i, :) * [1 t t^2 t^3].';

    if(i < size(vias, 1)-1)
        spline_k_plus_1 = A(i+1, :) * [1 t t^2 t^3].';
    end
    
    multipointProblem{ end + 1 } =  subs(spline_k, t, vias(i, 1)) == vias(i, 2);
    multipointProblem{ end + 1 } =  subs(spline_k, t, vias(i+1, 1)) == vias(i+1, 2);

    if(i < size(vias, 1)-1)
        multipointProblem{ end + 1 } =  subs(gradient(spline_k_plus_1, t), t, vias(i+1, 1)) == vias(i+1, 3);
        multipointProblem{ end + 1 } =  subs(gradient(spline_k, t), t, vias(i+1, 1)) == vias(i+1, 3);
    end
end

% initial/final velocity constraints
multipointProblem{ end + 1 } = subs(gradient(A(1, :) * [1 t t^2 t^3].', t), t, vias(1, 1)) == dqi;
multipointProblem{ end + 1 } = subs(gradient(A(end, :) * [1 t t^2 t^3].', t), t, vias(end, 1)) == dqf;

multipointSol = solve([multipointProblem{:}]);

splines = reshape(struct2array(multipointSol), 4, size(vias, 1)-1).' * [1;t;t^2;t^3];

figure(3);
subplot(131);
hold on;
scatter(vias(:, 1), vias(:, 2));
for i=1:size(vias, 1)-1
    fplot(splines(i), [vias(i,1) vias(i+1,1)]);
end
title('Multipoint splines (cont. acceleration)', 'q(t)')

subplot(132);
hold on;
for i=1:size(vias, 1)-1
    fplot(gradient(splines(i),t), [vias(i,1) vias(i+1,1)]);
end
title('Multipoint splines (cont. acceleration)', 'dq(t)');

subplot(133);
hold on;
for i=1:size(vias, 1)-1
    fplot(gradient(gradient(splines(i),t)), [vias(i,1) vias(i+1,1)]);
end
title('Multipoint splines (cont. acceleration)', 'ddq(t)');

