function [xEll,yEll,zEll] = genSurfXyzEllipse(covMat,mu,level)
    %GENSURFXYZELLIPSE
    %
    % [xEll,yEll,zEll] = GENSURFXYZELLIPSE(covMat,meanPts,level)
    %
    % covMat  - [3,3] array.
    % mu - length 3 vector.
    % level   - scalar. defaults to 3^2.
    %
    % xEll    - 
    % yEll    - 
    % zEll    - 

    if nargin < 3
        level = 3^2; % three sigma
    end
    
    R = chol(inv(level*covMat)); 
    [xSph,ySph,zSph] = sphere(10); 
    n = size(xSph,1);
    ptsSph = [xSph(:)'; ySph(:)'; zSph(:)'];
    ptsEll = R\ptsSph;
    
    mu = flipVecToColumn(mu);
    ptsEll = bsxfun(@plus,ptsEll,mu);
    
    xEll = reshape(ptsEll(1,:),n,n);
    yEll = reshape(ptsEll(2,:),n,n);
    zEll = reshape(ptsEll(3,:),n,n);
end

