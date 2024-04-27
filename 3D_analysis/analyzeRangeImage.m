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
pause(0.5);
imshow(rgbImg);
title('Original img');

pc = pcfromdepth(depthImg,1,intr);

detphMaxThreshold = 1300;
depthMinThreshold = 1000;
yMaxThreshold = 350;

% manually detect location of the forniture
pcRegion = pc.select( pc.Location(:, :, 2) < yMaxThreshold & pc.Location(:, :, 3) < detphMaxThreshold & pc.Location(:,:, 3) > depthMinThreshold);

figure(2);
pause(0.5);
pcshow(pcRegion);
title('Selected region');

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
pause(0.5);
imshow(dImg);
title('Binarized image w/region props');

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

%% centroid and 3D border points
uCentroid = round(regionProps.Centroid(2));
vCentroid = round(regionProps.Centroid(1));
centroidLocation = pc.Location(uCentroid,vCentroid, :);
centroidLocation = centroidLocation(1, :);

figure(4);
pause(0.5);
%pcshow(pcRegion);
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
subplot(1,2,1);
pause(0.5);
hold on;
pcshow(pcRegion);
plot(fittedPlane);
quiver3(centroidLocation(1,1),centroidLocation(1,2),centroidLocation(1,3), fittedPlane.Normal(1), fittedPlane.Normal(2), fittedPlane.Normal(3), "filled", "LineWidth",3,"AutoScaleFactor", 100 );
title('Plane fitting w/pcfitplane');

% manual 

centrCoords = pcRegion.Location - repmat(centroidLocation, pcRegion.Count, 1);
covMat = centrCoords'*centrCoords;
[v, a] = eig(covMat);

[amin, minIdx]=min(diag(a));
vmin = v(:, minIdx);
planeNorm = vmin/norm(vmin);
%planeD = planeNorm'*centroidLocation(1,:);

w = null(vmin');
[P,Q] = meshgrid(-300:300);
X = centroidLocation(1,1)+w(1,1)*P+w(1,2)*Q; % Compute the corresponding cartesian coordinates
Y = centroidLocation(1,2)+w(2,1)*P+w(2,2)*Q; %   using the two vectors in w
Z = centroidLocation(1,3)+w(3,1)*P+w(3,2)*Q;
subplot(1,2,2);
pause(0.5);
hold on;
pcshow(pcRegion);
quiver3(centroidLocation(1,1),centroidLocation(1,2),centroidLocation(1,3), vmin(1), vmin(2), vmin(3), "filled", "LineWidth",3,"AutoScaleFactor", 100 );
surf(X,Y,Z,  'EdgeColor','r');
title('Manual Plane fitting');

%% 

figure(6);
hold on;
pcOneBoundary = pcBoundary.select(pcBoundary.Location(:,1) > 300);

pcshow(pcOneBoundary);
lineModel = LineFitting3D(pcOneBoundary.Location);

%lt = -1000:1:1000; 
%x = centroidLocation(1) + lineModel(1)*t';
%y = centroidLocation(2) + lineModel(2)*t';
%z = centroidLocation(3) + lineModel(3)*t';

hold on;
quiver3(lineModel.Origin(1),lineModel.Origin(2),lineModel.Origin(3), lineModel.Normal(1), lineModel.Normal(2), lineModel.Normal(3), "filled", "LineWidth",3,"AutoScaleFactor", 100 );


%% 3D border fitting

fitLineFcn = @(points) LineFitting3D(points);
evalLineFcn =  @(model, points) ModelEval(model, points);

% a smart initial choice must be done
pcBoundarySubset = pcBoundary.select(pcBoundary.Location(:, 1) > centroidLocation(1));

boundaryLines = 4;
lineModels = cell(1,boundaryLines);

for i=1:1:boundaryLines
    % runs RANSAC on a subset of the boudary points
    [lineModel, inliersIdx] = ransac(pcBoundarySubset.Location,fitLineFcn,evalLineFcn, 8, 100);
    lineModels{i} = lineModel;
    
    if i == 1
        pcBoundarySubset = pointCloud(setdiff(pcBoundary.Location, pcBoundarySubset.Location(inliersIdx, :), 'rows', 'stable'));
    else
        pcBoundarySubset = pointCloud(setdiff(pcBoundarySubset.Location, pcBoundarySubset.Location(inliersIdx, :), 'rows', 'stable'));    
    end
end

figure(7);
pause(0.5);
hold on;
pcshow(pcBoundary);

for i=1:1:size(lineModels, 2)
    PlotLineModel(lineModels{i});
end


linesCombinations = 1:boundaryLines;
linesCombinations = nchoosek(linesCombinations,2);

angles = zeros(size(linesCombinations, 1),1);

for i=1:size(linesCombinations, 1)
    angles(i)=AngleBetweenLines(lineModels{linesCombinations(i,1)},lineModels{linesCombinations(i,2)});
end

[sortedAngles, sortAnglesIdx] = sort(angles, 'descend');

anglePoints = zeros(boundaryLines, 3);

for i=1:boundaryLines
    anglePoints(i,:) = LinesIntersection(lineModels{linesCombinations(sortAnglesIdx(i),1)}, lineModels{linesCombinations(sortAnglesIdx(i),2)}) ;
end

scatter3(anglePoints(:,1),anglePoints(:,2),anglePoints(:,3), 'filled', 'LineWidth', 20);


function distances = ModelEval(model, points)
    distances = zeros(size(points,1),1);

    for i=1:1:size(points,1)
        distances(i) = norm(points(i,:) - (model.Origin + PointToLineProjection(model,points(i, :)))).^2;
    end

    %disp(min(distances));
    %figure(10);
    %pause(0.5);
    %pcshow(pointCloud(points));
    %quiver3(model.Origin(1,1),model.Origin(1,2),model.Origin(1,3), model.Normal(1), model.Normal(2), model.Normal(3), "filled", "LineWidth",3,"AutoScaleFactor", 100 );
end

