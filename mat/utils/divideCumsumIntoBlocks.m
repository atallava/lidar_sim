function blockIds = divideCumsumIntoBlocks(cumsumVec,blockSize)
    %DIVIDECUMSUMINTOBLOCKS
    %
    % blockIds = DIVIDECUMSUMINTOBLOCKS(cumsumVec,blockSize)
    %
    % cumsumVec -
    % blockSize -
    %
    % blockIds  -
    
    u = floor(cumsumVec/blockSize);
    flag = logical(diff(u));
    nBlocks = sum(flag);
    blockIds = zeros(nBlocks,2);
    blockIds(:,2) = find(flag);
    blockIds(2:end,1) = blockIds(1:end-1,2)+1;
    blockIds(1,1) = 1;
end