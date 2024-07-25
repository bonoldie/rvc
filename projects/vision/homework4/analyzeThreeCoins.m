close all;
clear all;
clc;

%% Load img 
coinsImg = imread('eight.tif');

threshold = graythresh(coinsImg);
binImg = ~imbinarize(coinsImg, threshold);

% remove small areas
binImg = bwareaopen(binImg, 10);
connComps = bwconncomp(binImg, 8);

for i=1:1:connComps.NumObjects
    subplot(round(sqrt(connComps.NumObjects)),ceil(sqrt(connComps.NumObjects)),i);
    componentImg = false(size(binImg));
    componentImg(connComps.PixelIdxList{i}) = true;
    imshow(componentImg);
end
