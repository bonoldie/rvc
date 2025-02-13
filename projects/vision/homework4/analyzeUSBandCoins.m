close all;
clear all;
clc;

msg = "Img to analyze";
opts = ["Coins" "Chargers"];
choice = menu(msg,opts);

if choice == 1
    originalImg = rgb2gray(imread('mycoins.jpg')); 
elseif choice == 2
    originalImg = rgb2gray(imread('chargers.jpeg')); 
end

threshold = graythresh(originalImg);
binImg = imbinarize(originalImg, threshold);
    
binImg = bwareaopen(binImg, 1000);
connComps = bwconncomp(binImg, 8);
regionprops("table", connComps, "MajorAxisLength", "MinorAxisLength", "Centroid", "Area")

figure(1);
title('Components');

for i=1:1:connComps.NumObjects
    subplot(round(sqrt(connComps.NumObjects)),ceil(sqrt(connComps.NumObjects)),i);
    componentImg = false(size(binImg));
    componentImg(connComps.PixelIdxList{i}) = true;
    imshow(componentImg);
end

connCompsProps = regionprops(connComps, "Centroid", "MajorAxisLength", "MinorAxisLength", "Area", "Eccentricity", "BoundingBox");

compsRadiuses = mean([cat(1, connCompsProps.MajorAxisLength) cat(1, connCompsProps.MinorAxisLength)], 2) / 2;
compsArea = sort(cat(1, connCompsProps.Area), "ascend");
compsCentroids = cat(1, connCompsProps.Centroid);

pause(1);
figure(2);
imshow(originalImg);
title("Analysis result");
hold on;

plot(compsCentroids(:, 1), compsCentroids(:, 2), ".", "MarkerSize", 25);

for i = 1:connComps.NumObjects
    text(compsCentroids(i, 1), compsCentroids(i, 2), num2str(" " + i), "FontSize", 25);

    if connCompsProps(i).Eccentricity < 0.5
        % circle case
        viscircles(compsCentroids(i, :), compsRadiuses(i, :));
    else
        % square case 
        rectangle("Position",connCompsProps(i).BoundingBox, "LineWidth", 2, "EdgeColor", "r");        
    end
    
end

