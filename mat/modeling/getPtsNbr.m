function ptsNbr = getPtsNbr(pts,pt,maxDist,numNbrs)
    %GETPTSNBR
    %
    % ptsNbr = GETPTSNBR(pts,pt,maxDist)
    %
    % pts     -
    % pt      -
    % maxDist -
    %
    % ptsNbr  -
    
    % knnsearch becuase too many pts for pdist2
    
    % todo: this is a hacked parameter
    % find a cleaner algorithm
    if nargin < 4
        numNbrs = 100;
    end
    [idx,D] = knnsearch(pts,pt,'K',numNbrs);
    
    flag = (D <= maxDist);
    idx = idx(flag); 
    ptsNbr = pts(idx,:);
end