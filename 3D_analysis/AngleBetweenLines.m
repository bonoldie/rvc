function angle = AngleBetweenLines(lineModelA, lineModelB)
    angle = acos(dot(lineModelA.Normal, lineModelB.Normal));
end
