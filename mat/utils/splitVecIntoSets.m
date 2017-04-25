function setCell = splitVecIntoSets(vec,maxElementsPerSet)
    nSets = ceil(length(vec)/maxElementsPerSet);
    setCell = cell(1,nSets);
    for i = 1:(nSets-1)
        id1 = (i-1)*maxElementsPerSet+1;
        id2 = i*maxElementsPerSet;
        setCell{i} = vec(id1:id2);
    end
    id1 = (nSets-1)*maxElementsPerSet+1;
    id2 = length(vec);
    setCell{nSets} = vec(id1:id2);
end