function tape = calcPtsCellTape(ptsCell)
    %CALCPTSCELLTAPE
    %
    % tape = CALCPTSCELLTAPE(ptsCell)
    %
    % ptsCell - length nSegments cell. ptsCell{i} is a 2d array.
    %
    % tape    - [1,nSegments] vector.
    
    nSegments = length(ptsCell);
    % xy of pts centroids
    xyCentroids = zeros(nSegments,2);
    for i = 1:nSegments
        thisCentroid = mean(ptsCell{i},1);
        xyCentroids(i,:) = thisCentroid(1:2);
    end
    tape = calcPtsTape(xyCentroids);
end