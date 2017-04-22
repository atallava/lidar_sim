function nElementsPerClass = calcNElementsPerClass(elementIdsCell)
    %CALCNELEMENTSPERCLASS
    %
    % nElementsPerClass = CALCNELEMENTSPERCLASS(elementIdsCell)
    %
    % elementIdsCell    -
    %
    % nElementsPerClass -
    
    nClasses = length(elementIdsCell);
    nElementsPerClass = zeros(1,nClasses);
    for i = 1:nClasses
        nElementsPerClass(i) = length(elementIdsCell{i});
    end
end
   