close all;
clear;
clc;

%% Setup 

% images are captured using the primesense carmine 
% internals
f = 525;
u0 = 319.5;
v0 = 239.5;

% cameraIntrinsics object holds information about camera's intrinsic calibration parameters
intr = cameraIntrinsics(f, [u0,v0], [480 640]);

% load the images
rgbImg = imread("rgb_image.jpg");
depthImg = imread("range_image.png");

figure(1);
pause(0.5);
imshow(rgbImg);
title('Original img');

%% Point cloud extraction from range map
pc = pcfromdepth(depthImg,1,intr);

detphMaxThreshold = 1300;
depthMinThreshold = 1000;
yMaxThreshold = 350;

% An initial selection is made based on and estimation of the detph of the
% plane representing the front of the forniture

pcRegion = pc.select( pc.Location(:, :, 2) < yMaxThreshold & pc.Location(:, :, 3) < detphMaxThreshold & pc.Location(:,:, 3) > depthMinThreshold);
% pcRegion = pc.select( pc.Location(:, :, 3) < detphMaxThreshold & pc.Location(:,:, 3) > depthMinThreshold);

figure(2);
pause(0.5);
pcshow(pcRegion);
title('3D region');

%% 2D/3D image features analysis
dImg = depthImg;

rowMaxThreshold = 10000;%350;

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

%pcRegion = pcfromdepth(double(dImg) .* double(depthImg), 1, intr);
%pcRegion = pcRegion.select( pcRegion.Location(:, :, 3) < detphMaxThreshold & pcRegion.Location(:,:, 3) > depthMinThreshold);


el = strel('disk', 15);
dImg = imopen(dImg, el);
dImg = bwareaopen(dImg,50000);

figure(3);
pause(0.5);

subplot(121);
hold on;
imshow(dImg);
th = title('2D region props', Color='w');

boundaries = bwboundaries(dImg);
boundary = boundaries{1};


plot(boundary(:,2), boundary(:,1), 'r', 'LineWidth', 2, 'Color', 'yellow'); 

regionProps = regionprops(dImg, 'Centroid', 'BoundingBox', 'Orientation', 'Extrema');

% plot the props
rectangle('Position',regionProps.BoundingBox, 'LineWidth',0.5, 'EdgeColor', 'green');
scatter(regionProps.Extrema(:, 1), regionProps.Extrema(:, 2), 'filled', 'Color', 'r');
scatter(regionProps.Centroid(1), regionProps.Centroid(2), 'filled', 'Color', 'blue');
quiver(regionProps.Centroid(1), regionProps.Centroid(2), cos(deg2rad(regionProps.Orientation)), -sin(deg2rad(regionProps.Orientation)), 'Color', 'cyan', 'LineWidth', 2, 'AutoScaleFactor', 100);

% centroid and 3D border points
uCentroid = round(regionProps.Centroid(2));
vCentroid = round(regionProps.Centroid(1));
centroidLocation = pc.Location(uCentroid,vCentroid, :);
centroidLocation = centroidLocation(1, :);


subplot(122);
hold on;
scatter3(centroidLocation(1),centroidLocation(2),centroidLocation(3),'Color', 'blue', 'LineWidth', 20);
title('3D region props');

boundary3D = zeros(size(boundary, 1), 3);

% There may be another fancier method to select these points
for i=1:1:size(boundary)
    boundary3D(i, :) =  pc.Location(boundary(i,1), boundary(i,2), :);
end

pcBoundary = pointCloud(boundary3D);

% set border pc to red
pcBoundary.Color = uint8(zeros(pcBoundary.Count,3));
pcBoundary.Color(:,1) = 255;
pcBoundary.Color(:,2) = 0;
pcBoundary.Color(:,3) = 0;

pcshow(pcBoundary,'ColorSource','Color', 'MarkerSize', 20);

%% plane fitting

% w/pcfitplane
fittedPlane = pcfitplane(pcRegion, 1);

figure(5);
subplot(121);
pause(0.5);
hold on;
pcshow(pcRegion);
plot(fittedPlane);
quiver3(centroidLocation(1,1),centroidLocation(1,2),centroidLocation(1,3), fittedPlane.Normal(1), fittedPlane.Normal(2), fittedPlane.Normal(3), "filled", "LineWidth",3,"AutoScaleFactor", 100 );
title('Plane fitting w/pcfitplane');

% manual plane fitting
planeComponents = FitPlane(pcRegion);

[P,Q] = meshgrid(-300:300);
X = centroidLocation(1,1)+planeComponents(1,1)*P+planeComponents(1,2)*Q; % Compute the corresponding cartesian coordinates
Y = centroidLocation(1,2)+planeComponents(2,1)*P+planeComponents(2,2)*Q; % using the two vectors in w
Z = centroidLocation(1,3)+planeComponents(3,1)*P+planeComponents(3,2)*Q;

subplot(122);
pause(0.5);
hold on;
pcshow(pcRegion);
surf(X,Y,Z,  'EdgeColor','r');

planeNormal = cross(planeComponents(:,1),planeComponents(:,2));
planeNormal = planeNormal ./ norm(planeNormal);

quiver3(centroidLocation(1,1),centroidLocation(1,2),centroidLocation(1,3), planeNormal(1), planeNormal(2), planeNormal(3), "filled", "LineWidth",3,"AutoScaleFactor", 100, Color='b');

p1 = pcRegion.Location(1000, :)';
d1 = PointToPlaneDistance(planeComponents, p1 - centroidLocation');

scatter3(p1(1), p1(2), p1(3), "LineWidth",3, Color='g');
quiver3(p1(1),  p1(2), p1(3), [d1 0 0]*planeNormal, [0 d1 0]*planeNormal, [0 0 d1]*planeNormal, "filled", "LineWidth",3, Color='g');

title('Manual Plane fitting');

%% 3D border fitting

fitLineFcn = @(points) LineFitting3D(points);
evalLineFcn =  @(model, points) ModelEval(model, points);

pcBoundarySubset = pcBoundary;%  we may want a smart initial choice: .select(pcBoundary.Location(:, 1) > centroidLocation(1));

% We want to extract the lines representing the 4 sides
boundaryLines = 4;
lineModels = cell(1,boundaryLines);

for i=1:1:boundaryLines
    % runs RANSAC on a subset of the boudary points
    [lineModel, inliersIdx] = ransac(pcBoundarySubset.Location,fitLineFcn,evalLineFcn, 8, 250, MaxNumTrials=10000);
    lineModels{i} = lineModel;

    % Updates the point cloud of boundary points removing the inliers of the current line model
    pcBoundarySubset = pointCloud(setdiff(pcBoundarySubset.Location, pcBoundarySubset.Location(inliersIdx, :), 'rows', 'stable'));    
end

figure(7);
pause(0.5);
hold on;
pcshow(pcBoundary);

for i=1:1:size(lineModels, 2)
    linePlot =PlotLineModel(lineModels{i});
    linePlot.DisplayName = "BoundaryLine";
end

linesCombinations = 1:boundaryLines;
linesCombinations = nchoosek(linesCombinations,2);

angles = zeros(size(linesCombinations, 1),1);

for i=1:size(linesCombinations, 1)
    angles(i) = AngleBetweenLines(lineModels{linesCombinations(i,1)},lineModels{linesCombinations(i,2)});
end

[sortedAngles, sortAnglesIdx] = sort(angles, 'descend');

anglePoints = zeros(boundaryLines, 3);

for i=1:boundaryLines
    anglePoints(i,:) = LinesIntersection(lineModels{linesCombinations(sortAnglesIdx(i),1)}, lineModels{linesCombinations(sortAnglesIdx(i),2)}) ;
end

scatter3(anglePoints(:,1),anglePoints(:,2),anglePoints(:,3), 'filled', 'LineWidth', 30, DisplayName='3D lines intersection points');

lh = legend();
set( lh, 'Box', 'on', 'Color', [0.9,0.9,0.9], 'EdgeColor', get( lh, 'Color' )) ;

% Evaluate how well the line model represent the underlying point cloud
function distances = ModelEval(model, points)
    distances = zeros(size(points,1),1);

    for i=1:1:size(points,1)
        distances(i) = norm(points(i,:) - (model.Origin + PointToLineProjection(model,points(i, :)))).^2;
    end
end
