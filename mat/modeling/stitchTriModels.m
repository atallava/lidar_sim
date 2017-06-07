function triModels = stitchTriModels(triModelCell)
    %STITCHTRIMODELS
    %
    % triModels = STITCHTRIMODELS(triModelCell)
    %
    % triModelCell -
    %
    % triModels    -
    
    triModels.tri = [];
    triModels.ptsFit = [];
    rangeVars = [];
    triModels.hitProbVec = [];
        
    for i = 1:length(triModelCell)
        thisModel = triModelCell{i};
        
        triModels.tri = [triModels.tri; ...
            thisModel.tri + size(triModels.ptsFit,1)];
        triModels.ptsFit = [triModels.ptsFit; thisModel.ptsFit];
        rangeVars = [rangeVars thisModel.rangeVar];
        triModels.hitProbVec = [triModels.hitProbVec thisModel.hitProbVec];
    end
    
    triModels.rangeVar = mean(rangeVars);
end