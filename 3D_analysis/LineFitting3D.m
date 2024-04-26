function line = LineFitting3D(points)
    normCoords = points - repmat(mean(points,1), size(points,1), 1);
    covMat = normCoords'*normCoords;
    [v, a] = eig(covMat);

    [~, maxIdx]=max(diag(a));
    line = v(:, maxIdx);
end