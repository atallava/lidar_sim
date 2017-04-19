function pts = stitchPtsCell(ptsCell)
    pts = [];
    for i = 1:length(ptsCell)
        pts = [pts; ptsCell{i}];
    end
end