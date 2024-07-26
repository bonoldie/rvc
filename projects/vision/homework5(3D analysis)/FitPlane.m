function [planeComponents,planeNorm ] = FitPlane(pointCloud)
    centrCoords = pointCloud.Location - mean(pointCloud.Location);
    covMat = centrCoords'*centrCoords;
    [v, a] = eig(covMat);
    
    [amin, minIdx]=min(diag(a));
    vmin = v(:, minIdx);
    planeNorm = vmin/norm(vmin);
    % planeD = planeNorm'*centroidLocation(1,:);
    
    planeComponents = null(vmin');
end
