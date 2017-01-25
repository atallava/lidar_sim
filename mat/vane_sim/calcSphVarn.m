function sphVarn = calcSphVarn(pts)
    covMat = cov(pts);
    e = eig(covMat);
    
    condn = all(e > 0);
    msg = sprintf('%s: eigenvalues not all > 0.',mfilename);
    assert(condn,msg);
    
    sphVarn = min(e)/sum(e);
end