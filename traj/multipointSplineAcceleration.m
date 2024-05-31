close all;
clear all;
clc;

vias = [
    0, 0;
    1, 1;
    2, 5;
    5, 10;
    10, -20;
    15, 0;
    20, 15;
];

vi = 0;
vf = 0;

disp(array2table(vias, 'VariableNames', {'t', 'q'}))
disp('-------------')

syms t 
A = sym('a',[size(vias, 1)-1 4]);

multipointProblem = {};

% continuity constraints
    for i=1:size(vias, 1)-1
    spline_k = A(i, :) * [1 t t^2 t^3].';

    multipointProblem{ end + 1 } =  subs(spline_k, t, vias(i+1, 1)) == vias(i+1, 2);
    
    if(i < size(vias, 1)-1)
        spline_k_plus_1 = A(i+1, :) * [1 t t^2 t^3].';

        multipointProblem{ end + 1 } =  subs(spline_k, t, vias(i+1, 1)) == subs(spline_k_plus_1, t, vias(i+1, 1));
        multipointProblem{ end + 1 } =  subs(gradient(spline_k, t), t, vias(i+1, 1)) == subs(gradient(spline_k_plus_1, t), t, vias(i+1, 1));
        multipointProblem{ end + 1 } =  subs(gradient(gradient(spline_k, t),t), t, vias(i+1, 1)) == subs(gradient(gradient(spline_k_plus_1, t),t), t, vias(i+1, 1));
    end
end

% initial velocity and position constraints 
multipointProblem{ end + 1 } = subs(A(1, :) * [1 t t^2 t^3].', t, vias(1, 1)) == vias(1, 2);
multipointProblem{ end + 1 } = subs(gradient(A(1, :) * [1 t t^2 t^3].',t), t, vias(1, 1)) == vi;

% final velocity constraints
multipointProblem{ end + 1 } = subs(gradient(A(end, :) * [1 t t^2 t^3].',t), t, vias(end, 1)) == vf;

multipointSol = solve([multipointProblem{:}]);

splines = reshape(struct2array(multipointSol), 4, size(vias, 1)-1).' * [1 t t^2 t^3].';

pretty(splines);

figure(1);
hold on;
scatter(vias(:, 1), vias(:, 2));
for i=1:size(vias, 1)-1
    fplot(splines(i), [vias(i,1) vias(i+1,1)]);
end
title('Multipoint splines position')

figure(2);
hold on;
for i=1:size(vias, 1)-1
    fplot(gradient(splines(i),t), [vias(i,1) vias(i+1,1)]);
end
title('Multipoint splines velocity')

figure(3);
hold on;
for i=1:size(vias, 1)-1
    fplot(gradient(gradient(splines(i),t),t), [vias(i,1) vias(i+1,1)]);
end
title('Multipoint splines acceleration')
