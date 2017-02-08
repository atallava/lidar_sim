function phraseCell = printVecsAsStlInput(vecCell)
    for i = 1:length(vecCell)
        phraseCell{i} = printVecAsStlInput(vecCell{i});
    end
end