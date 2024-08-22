clear all;
close all;
clc;

%%% load data

fid = fopen('cam2target.csv', 'r');
A =  textscan(fid, '%f,%f,%f,%f,%f,%f,%f');

% rotations
rvec_t_c = [A{2}, A{3}, A{4}];
% translations
tvec_t_c = [A{5}, A{6}, A{7}];

fid = fopen('gripper2base.csv', 'r');
A =  textscan(fid, '%f,%f,%f,%f,%f,%f,%f');

% rotations
rvec_b_g = [A{2}, A{3}, A{4}];
% translations
tvec_b_g = [A{5}, A{6}, A{7}];

N_samples = size(rvec_b_g, 1);

%%% setup rototranslation matrices

% base to gripper roto-translations
Transforms_b_g = zeros(4, 4, N_samples); 

% target to camera roto-translations
Transforms_t_c = zeros(4, 4, N_samples); 

for i=1:N_samples
    Transforms_b_g(:,:,i) = [Rodrigues(rvec_b_g(i,:)'), tvec_b_g(i, :)'; 0 0 0 1];
    Transforms_t_c(:,:,i) = [Rodrigues(rvec_t_c(i,:)'), tvec_t_c(i, :)'; 0 0 0 1];
end

%This is the Tsai method as described in the original paper:
% [cam2grip, err1] = TSAIleastSquareCalibration(Transforms_b_g, Transforms_t_c);


% Camera i to j (i+1 in this case) roto-translations
Transforms_camera_i_j = [];

% Gripper i to j (i+1 in this case) roto-translations
Transforms_gripper_i_j = [];

for k = 1:N_samples-1
    i = k;
    j = k+1;

    Transforms_camera_i_j = [Transforms_camera_i_j, inv(Transforms_t_c(:,:,i)) * Transforms_t_c(:,:,j)];
    Transforms_gripper_i_j = [Transforms_gripper_i_j, inv(Transforms_b_g(:,:,i)) * Transforms_b_g(:,:,j)];
end

Transform_g_c = tsai(Transforms_gripper_i_j, Transforms_camera_i_j);

figure(5);
subplot(121);
plotRef(eye(3), [0,0,0]', 'Base');
hold on;
grid on;
title('Base to target');

for i=1:N_samples
    plotRef(Transforms_b_g(1:3,1:3, i), Transforms_b_g(1:3, 4, i), num2str(i));

    Transform_b_c = Transforms_b_g(:,:,i) * Transform_g_c;
    plotRef(Transform_b_c(1:3,1:3), Transform_b_c(1:3, 4), num2str(i));

    Transform_b_t = Transform_b_c * inv(Transforms_t_c(:,:,i));
    plotRef(Transform_b_t(1:3,1:3), Transform_b_t(1:3, 4), 'Target');
end

subplot(122);
plotRef(eye(3), [0,0,0]', 'Target');
hold on;
grid on;
title('Target to base');

for i=1:N_samples
    plotRef(Transforms_t_c(1:3,1:3, i), Transforms_t_c(1:3, 4, i), num2str(i));

    Transform_t_g = Transforms_t_c(:,:,i) * inv(Transform_g_c);
    plotRef(Transform_t_g(1:3,1:3), Transform_t_g(1:3, 4), num2str(i));

    Transform_t_b = Transform_t_g * inv(Transforms_b_g(:,:,i));
    plotRef(Transform_t_b(1:3,1:3), Transform_t_b(1:3, 4), 'Base');
end

