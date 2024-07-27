clear all;
close all;
clc;

% Hand-eye calibration  
%
% cam2target.csv format: 1 index, 2-4 rvec, 5-7 tvec. 
% cam2target, transfer a point in camera frame to target(chessboard) frame.
% gripper2base, transfer a point in gripper frame to base frame.
% rvec: the axis-angle notation. rvec's direction is rotation direction, norm is the rotation angle (in rad)
% tvec: unit: mm
% 
% Dong Yan  2022.01.4
%
% Modified by Umberto Castellani June 2024
% Extended by Enrico Bonoldi July 2024

%% load data
% these files contains the 
fid = fopen('cam2target.csv', 'r');
A =  textscan(fid, '%f,%f,%f,%f,%f,%f,%f');
rvec_t_c = [A{2}, A{3}, A{4}];
tvec_t_c = [A{5}, A{6}, A{7}];

fid = fopen('gripper2base.csv', 'r');
A =  textscan(fid, '%f,%f,%f,%f,%f,%f,%f');
rvec_b_g = [A{2}, A{3}, A{4}];
tvec_b_g = [A{5}, A{6}, A{7}];

N_samples = size(rvec_b_g, 1);


