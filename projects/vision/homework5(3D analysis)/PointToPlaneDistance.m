function d = PointToPlaneDistance(planeComponents, point)
    planeNorm = cross(planeComponents(:,1),planeComponents(:,2));
    proj = dot(planeNorm,point);
    d =  norm(proj);
end