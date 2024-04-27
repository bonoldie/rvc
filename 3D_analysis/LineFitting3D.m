function lineModel = LineFitting3D(points)
    centroid = mean(points,1);
    normCoords = points - repmat(centroid, size(points,1), 1);
    covMat = normCoords'*normCoords;
    [v, a] = eig(covMat);

    [~, maxIdx]=max(diag(a));
    lineModel = struct('Normal',v(:, maxIdx)','Origin', centroid);
end