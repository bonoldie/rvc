clear all;
close all;
clc;

points = [
    0, 0, 0;
    1, 0, 0;
    2, 1, 0;
    2, 1, 2;
    2, 0, 2
];

centers = [
    1, 1, 0;
    2, 1, 1
];

segmetType = [
    ''
] 

%% first line
s = linspace(0, norm(points(2, :) - points(1, :), 30));
line_1 = points(1, :) + s' * (points(2, :) - points(1, :)) / (norm(points(2, :) - points(1, :)));

plot3(line_1(:, 1), line_1(:, 2), line_1(:, 3), "LineWidth", 2); hold on; grid on;

%% first circle
theta = linspace(0, -pi / 2, 100);
radius_1 = norm(points(2, :) - centers(1, :));
x = centers(1, 1) + radius_1 * cos(theta);
y = centers(1, 2) + radius_1 * sin(theta);
z = centers(1, 3) + zeros(size(theta));

plot3(x, y, z, "LineWidth", 2);

%% second circle
theta = linspace(0, pi, 100);

radius_2 = norm(points(3, :) - centers(2, :));
x = centers(2, 1) + zeros(size(theta));
y = centers(2, 2) + radius_2 * sin(theta);
z = centers(2, 3) + radius_2 * cos(theta);

plot3(x, y, z, "LineWidth", 2);

%% last line
s = linspace(0, norm(points(5, :) - points(4, :), 100));
line_2 = points(4, :) + s' * (points(5, :) - points(4, :)) / (norm(points(5, :) - points(4, :)));
plot3(line_2(:, 1), line_2(:, 2), line_2(:, 3), "LineWidth", 2);

%% plotting frame
axes_length = 0.2;

% Assi del sistema di riferimento
quiver3(points(1, 1), points(1, 2), points(1, 3), axes_length, 0, 0, 'r', 'LineWidth', 1);
quiver3(points(1, 1), points(1, 2), points(1, 3), 0, axes_length, 0, 'r', 'LineWidth', 1);
quiver3(points(1, 1), points(1, 2), points(1, 3), 0, 0, axes_length, 'r', 'LineWidth', 1);
text(axes_length, 0, 0, 'X', 'FontSize', 8, 'VerticalAlignment', 'baseline');
text(0, axes_length, 0, 'Y', 'FontSize', 8, 'VerticalAlignment', 'baseline');
text(0, 0, axes_length, 'Z', 'FontSize', 8, 'VerticalAlignment', 'baseline');

quiver3(points(2, 1), points(2, 2), points(2, 3), axes_length, 0, 0, 'r', 'LineWidth', 1);
quiver3(points(2, 1), points(2, 2), points(2, 3), 0, axes_length, 0, 'r', 'LineWidth', 1);
quiver3(points(2, 1), points(2, 2), points(2, 3), 0, 0, axes_length, 'r', 'LineWidth', 1);
text(points(2, 1) + axes_length, 0, 0, 'X', 'FontSize', 8, 'VerticalAlignment', 'baseline');
text(points(2, 1), points(2, 2) + axes_length, 0, 'Y', 'FontSize', 8, 'VerticalAlignment', 'baseline');
text(points(2, 1), points(2, 2), points(2, 3) + axes_length, 'Z', 'FontSize', 8, 'VerticalAlignment', 'baseline');

quiver3(points(3, 1), points(3, 2), points(3, 3), axes_length, 0, 0, 'r', 'LineWidth', 1);
quiver3(points(3, 1), points(3, 2), points(3, 3), 0, axes_length, 0, 'r', 'LineWidth', 1);
quiver3(points(3, 1), points(3, 2), points(3, 3), 0, 0, axes_length, 'r', 'LineWidth', 1);
text(points(3, 1) + axes_length, points(3, 2), 0, 'X', 'FontSize', 8, 'VerticalAlignment', 'baseline');
text(points(3, 1), points(3, 2) + axes_length, 0, 'Y', 'FontSize', 8, 'VerticalAlignment', 'baseline');
text(points(3, 1), points(3, 2), points(3, 3) + axes_length, 'Z', 'FontSize', 8, 'VerticalAlignment', 'baseline');

quiver3(points(4, 1), points(4, 2), points(4, 3), axes_length, 0, 0, 'r', 'LineWidth', 1);
quiver3(points(4, 1), points(4, 2), points(4, 3), 0, -axes_length, 0, 'r', 'LineWidth', 1);
quiver3(points(4, 1), points(4, 2), points(4, 3), 0, 0, -axes_length, 'r', 'LineWidth', 1);
text(points(4, 1) + axes_length, points(4, 2), points(4, 3), 'X', 'FontSize', 8, 'VerticalAlignment', 'baseline');
text(points(4, 1), points(4, 2) - axes_length, points(4, 3), 'Y', 'FontSize', 8, 'VerticalAlignment', 'baseline');
text(points(4, 1), points(4, 2), points(4, 3) -axes_length, 'Z', 'FontSize', 8, 'VerticalAlignment', 'baseline');

quiver3(points(5, 1), points(5, 2), points(5, 3), axes_length, 0, 0, 'r', 'LineWidth', 1);
quiver3(points(5, 1), points(5, 2), points(5, 3), 0, -axes_length, 0, 'r', 'LineWidth', 1);
quiver3(points(5, 1), points(5, 2), points(5, 3), 0, 0, -axes_length, 'r', 'LineWidth', 1);
text(points(5, 1) + axes_length, points(5, 2), points(5, 3), 'X', 'FontSize', 8, 'VerticalAlignment', 'baseline');
text(points(5, 1), points(5, 2) - axes_length, points(5, 3), 'Y', 'FontSize', 8, 'VerticalAlignment', 'baseline');
text(points(5, 1), points(5, 2), points(5, 3) -axes_length, 'Z', 'FontSize', 8, 'VerticalAlignment', 'baseline');