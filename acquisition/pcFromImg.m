clear all;
clc;

% primesense carmine internals
f = 525;
u0 = 319.5;
v0 = 239.5;

img = imread("depth_image.png");

pc = zeros(size(img, 1),size(img, 2), 3);

for i = 1:size(img, 1)
    for j = 1:size(img, 2)
        pc(i, j, 1) = -(double(img(i, j))*(i- u0))/f;
        pc(i, j ,2) = -(double(img(i, j))*(j- v0))/f;
        pc(i, j, 3) = double(img(i, j));   
    end
end


pc = pointCloud([vec(pc(:,:,1)),vec(pc(:,:,2)), vec(pc(:,:,3))]);

pcRegion = pc.select(pc.Location(:,1) ~= 0);

pcRegion = pc.select(pc.Location(:,1) ~= 0 & abs(pc.Location(:, 1)) < 2000);
pcRegion = pcRegion.select(pcRegion.Location(:,2) ~= 0 & abs(pcRegion.Location(:, 2)) < 2000);
pcRegion = pcRegion.select(pcRegion.Location(:,3) ~= 0 & abs(pcRegion.Location(:, 3)) < 2000);

%pcshow(pcRegion);

pcDownSampled = pcdownsample(pcRegion,"gridAverage",10);


%% 

pcshow(pcDownSampled);

%mesh = pc2surfacemesh(pcDownSampled,"ball-pivot",30);
%surfaceMeshShow(mesh);

