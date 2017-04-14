function ellipsoidModels = stitchEllipsoidModels(ellipsoidModelCell)
    ellipsoidModels = ellipsoidModelCell{1};
    for i = 2:length(ellipsoidModelCell)
        ellipsoidModels = [ellipsoidModels ellipsoidModelCell{i}];
    end
end