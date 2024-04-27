function plottedLine = PlotLineModel(lineModel)
    t = -1000:1:1000; 
    x = lineModel.Origin(1) + lineModel.Normal(1)*t';
    y = lineModel.Origin(2) + lineModel.Normal(2)*t';
    z = lineModel.Origin(3) + lineModel.Normal(3)*t';
    plottedLine = line(x,y,z);
end
