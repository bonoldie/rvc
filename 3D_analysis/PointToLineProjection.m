function p = PointToLineProjection(lineModel, point)
    p = dot((point -  lineModel.Origin), lineModel.Normal)*lineModel.Normal;
end