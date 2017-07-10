function features = calcFeaturesForGroundSegmentation(pts)

    zCentroid = calcMeanHeight(pts);
    sphVarn = calcSphericalVarn(pts);
    extents = calcSortedPaxExtents(pts);
    
    features = [zCentroid sphVarn extents];
end

%% feature functions
function z = calcMeanHeight(pts)
    centroid = mean(pts,1);
    z = centroid(3);
end

function extents = calcSortedPaxExtents(pts)
    extents = calcPaxExtents(pts);
    extents = sort(extents);
end

function extents = calcPaxExtents(pts)
    nPts = size(pts,1);
    ptsXy = pts(:,1:2);
    ptsXyMean = mean(ptsXy,1);
    ptsXy = bsxfun(@minus,ptsXy,ptsXyMean);
    
    covMat = cov(ptsXy);
    [V,~] = eig(covMat);
   
    paxCoords = zeros(nPts,2);
    for i = 1:nPts
        for j = 1:2
            paxCoords(i,j) = dot(ptsXy(i,:),V(:,j));
        end
    end   
    
    extents = zeros(1,2);
    for i = 1:2
        extents(i) = range(paxCoords(:,i));
    end
end

function eigVals = calcEigVals(pts)
    covMat = cov(pts);
    eigVals = eig(covMat);
end

function sphVarn = calcSphericalVarn(pts)
    eigVals = calcEigVals(pts);
    sphVarn = min(eigVals)/sum(eigVals);
end