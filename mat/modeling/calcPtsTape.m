function tape = calcPtsTape(pts)
    %CALCPTSTAPE
    %
    % tape = CALCPTSTAPE(pts)
    %
    % pts  - [nPts,dimPts] array.
    %
    % tape - [1,nPts] array.
    
    nPts = size(pts,1);
    tape = zeros(1,nPts);
    tape(1) = 1;
    remPts = pts(2:end,:);
    remIds = 2:nPts;
    for i = 2:(nPts-1)
       nnId = knnsearch(remPts,pts(tape(i-1),:));
       remPts(nnId,:) = [];
       tape(i) = remIds(nnId);
       remIds(nnId) = [];
    end
    tape(end) = remIds(1);
end