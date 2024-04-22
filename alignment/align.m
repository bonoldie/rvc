close all;
clear all;
clc;

dirContent = dir("depth");
depthImages = sort(extractfield(dirContent,"name"));
intr = cameraIntrinsics([525 525],[319.5 239.5],[480 640]);

selectedImagesIdx = 3:1:7;
selectedImages = depthImages(selectedImagesIdx);
images = {};

pointClouds = {};
downsampledPointClouds = {};

transformations = {};
alignedPointClouds = {};

for i = 1:1:size(selectedImages,2)
    images{i} = imread(strcat("depth/", char(selectedImages(i))));
    pointClouds{i} = pcfromdepth(images{i}, 1, intr); 
    downsampledPointClouds{i} = pcdownsample( pointClouds{i} ,"random", 0.3);
end

finalPointCloud = downsampledPointClouds{1};
transformations{1} = rigidtform3d();

for i = 2:1:size(selectedImages,2)
    fprintf("ICP from %i to %i \n",i-1, i);
    transformations{i} = pcregistericp(downsampledPointClouds{i}, downsampledPointClouds{i-1}, Metric="pointToPlane");
end

%% Second part

cumulativeTransformation = transformations{1};
alignedPointClouds{1} = downsampledPointClouds{1};

for i = 2:1:size(selectedImages,2)
    cumulativeTransformation = rigidtform3d(cumulativeTransformation.A * transformations{i}.A);
    alignedPointClouds{i} = pctransform(downsampledPointClouds{i}, cumulativeTransformation);
end


figure(1);
hold on;
for i = 1:1:3
    pcshow(alignedPointClouds{i});
end


