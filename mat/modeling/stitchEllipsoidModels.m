function [ellipsoidModels,blockMembershipMap] = stitchEllipsoidModels(ellipsoidModelCell)
    %STITCHELLIPSOIDMODELS
    %
    % [ellipsoidModels,blockMembershipMap] = STITCHELLIPSOIDMODELS(ellipsoidModelCell)
    %
    % ellipsoidModelCell -
    %
    % ellipsoidModels    -
    % blockMembershipMap -
    
    ellipsoidModels = ellipsoidModelCell{1};
    blockMembershipMap = ones(1,length(ellipsoidModelCell{1}));
    for i = 2:length(ellipsoidModelCell)
        ellipsoidModels = [ellipsoidModels ellipsoidModelCell{i}];
        blockMembershipMap = [blockMembershipMap ...
            ones(1,length(ellipsoidModelCell{i}),1)*i];
    end
end