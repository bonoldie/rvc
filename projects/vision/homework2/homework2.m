close all;
clear all;
clc;

% primesense carmine internals
f = 525;
u0 = 319.5;
v0 = 239.5;

img1 = imread("range_image1.png");
img2 = imread("range_image2.png");

pc1 = zeros(size(img1, 1),size(img1, 2), 3);
pc2 = zeros(size(img2, 1),size(img2, 2), 3);

for i = 1:size(img1, 1)
    for j = 1:size(img1, 2)
        pc1(i, j, 1) = -(double(img1(i, j))*(i- u0))/f;
        pc1(i, j ,2) = -(double(img1(i, j))*(j- v0))/f;
        pc1(i, j, 3) = double(img1(i, j));   
    end
end

for i = 1:size(img2, 1)
    for j = 1:size(img2, 2)
        pc2(i, j, 1) = -(double(img2(i, j))*(i- u0))/f;
        pc2(i, j ,2) = -(double(img2(i, j))*(j- v0))/f;
        pc2(i, j, 3) = double(img2(i, j));   
    end
end

pc1 = pointCloud([reshape(pc1(:,:,1),[],1),reshape(pc1(:,:,2),[],1),reshape(pc1(:,:,3),[],1)]);
pcRegion1 = pc1;
% pcRegion1 = pc1.select(pc1.Location(:,1) ~= 0);

pc2 = pointCloud([reshape(pc2(:,:,1),[],1),reshape(pc2(:,:,2),[],1),reshape(pc2(:,:,3),[],1)]);
pcRegion2 = pc2;
% pcRegion2 = pc2.select(pc2.Location(:,1) ~= 0);

% Region cleanup

% pcRegion = pc.select(pc.Location(:,1) ~= 0);
% pcRegion = pc.select(pc.Location(:,1) ~= 0 & abs(pc.Location(:, 1)) < 2000);
% pcRegion = pcRegion.select(pcRegion.Location(:,2) ~= 0 & abs(pcRegion.Location(:, 2)) < 2000);
% pcRegion = pcRegion.select(pcRegion.Location(:,3) ~= 0 & abs(pcRegion.Location(:, 3)) < 2000);
% pcDownSampled = pcdownsample(pcRegion,"gridAverage",10);

% Display and save
pcwrite(pcRegion1,'range_image1.ply',Encoding='ascii');
pcwrite(pcRegion2,'range_image2.ply',Encoding='ascii');

figure(1);

subplot(221);
title('Range image');
imagesc(img1);
colorbar;

subplot(222);
title('Point cloud');
pcshow(pcRegion1);

subplot(223);
imagesc(img2);
colorbar;

subplot(224);
pcshow(pcRegion2);


