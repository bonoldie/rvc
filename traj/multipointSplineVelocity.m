close all;
clear all;
clc;

vias = [
    0, 0;
    1, 1;
    2, 5;
    5, 10;
    7, -2;
    10, -20;
    15, 0;
];

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

disp(array2table(vias, 'VariableNames', {'t', 'q', 'dq'}))
disp('-------------')

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

splines = reshape(struct2array(multipointSol), 4, size(vias, 1)-1).' * [1 t t^2 t^3].';

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
