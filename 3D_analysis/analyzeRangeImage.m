close all;
clear;
clc;

% images are captured using the primesense carmine 
% internals
f = 525;
u0 = 319.5;
v0 = 239.5;

intr = cameraIntrinsics(f, [u0,v0], [480 640]);

% load the images
rgbImg = imread("0000206-000006870780.jpg");
depthImg = imread("0000206-000006840696.png");
figure(1);
imshow(rgbImg);

pc = pcfromdepth(depthImg,1,intr);


detphMaxThreshold = 1300;
depthMinThreshold = 1000;
yMaxThreshold = 350;

% manually detect location of the forniture
pcRegion = pc.select( pc.Location(:, :, 2) < yMaxThreshold & pc.Location(:, :, 3) < detphMaxThreshold & pc.Location(:,:, 3) > depthMinThreshold);
figure(2);
pcshow(pcRegion);


% image analysis

dImg = depthImg;

rowMaxThreshold = 390;

for i=1:1:size(dImg,1)
    for j=1:1:size(dImg,2)
        if dImg(i,j) < depthMinThreshold || dImg(i,j) > detphMaxThreshold || i > rowMaxThreshold
            dImg(i,j) = 10000;
        end
    end
end

dImg = imadjust(dImg);
dImg = imbinarize(dImg);
dImg = ~dImg;

el = strel('disk', 5);
dImg = imopen(dImg, el);
dImg = bwareaopen(dImg,1000);

figure(3);
imshow(dImg);

boundaries = bwboundaries(dImg);
boundary = boundaries{1};

hold on;
plot(boundary(:,2), boundary(:,1), 'r', 'LineWidth', 2, 'Color', 'yellow'); 

regionProps = regionprops(dImg, 'Centroid', 'BoundingBox', 'Orientation', 'Extrema');

% plot the props
rectangle('Position',regionProps.BoundingBox, 'LineWidth',0.5, 'EdgeColor', 'green');
scatter(regionProps.Extrema(:, 1), regionProps.Extrema(:, 2), 'filled', 'Color', 'r');
scatter(regionProps.Centroid(1), regionProps.Centroid(2), 'filled', 'Color', 'blue');
quiver(regionProps.Centroid(1), regionProps.Centroid(2), cos(deg2rad(regionProps.Orientation)), -sin(deg2rad(regionProps.Orientation)), 'Color', 'cyan', 'LineWidth', 2, 'AutoScaleFactor', 100);


