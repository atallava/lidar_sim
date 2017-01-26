function sphVarn = calcSphericalVariation(pts)
    %CALCSPHERICALVARIATION
    %
    % sphVarn = CALCSPHERICALVARIATION(pts)
    %
    % pts     - [nPts,3] array.
    %
    % sphVarn - scalar.
    
    covMat = cov(pts);
    e = eig(covMat);
    
    condn = all(e > 0);
    msg = sprintf('%s: eigenvalues not all > 0.',mfilename);
    assert(condn,msg);
    
    sphVarn = min(e)/sum(e);
end