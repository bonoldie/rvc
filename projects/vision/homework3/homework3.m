close all;
clear all;
clc;

% Load point cloud extracted in the previous homework
pc = pcread('range_image1.ply');

original_img_width = 640;
original_img_height = 480;


% Restore the original size from the vectorized Locations in the point cloud
rawPcData = reshape(pc.Location, original_img_height, original_img_width, 3);

% Considers only a region of the point cloud 
height = 200;
width = 200;
pcPatch = rawPcData((original_img_height/2)-(height/2):(original_img_height/2)+height, (original_img_width/2)-(width/2):(original_img_width/2)+width,:);

figure(1);
pcshow(pc);
hold on;

for i = 2:2:height-1
    for j = 2:2:width-1
        for ii=-1:1:1
            for jj=-1:1:1
                if ~any(pcPatch(i,j,:)) || ~any(pcPatch(i+ii,j+jj))
                    continue;
                end

                if ii ~= 0 && jj ~= 0
                    plot3([pcPatch(i,j,1), pcPatch(i+ii,j+jj,1)],[pcPatch(i,j,2), pcPatch(i+ii,j+jj,2)],[pcPatch(i,j,3), pcPatch(i+ii,j+jj,3)], LineWidth=0.5);
                end
            end
        end
    end
end

camzoom(36);
view(100,100);
camtarget(reshape(pcPatch((height/2),(width/2),:), 1, 3));
