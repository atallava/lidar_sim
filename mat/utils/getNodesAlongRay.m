function nodesAlongRay = getNodesAlongRay(rayOrigin,rayDirn,minDist,maxDist,step)
    rayDirn = flipVecToRow(rayDirn);
    distsAlongRay = minDist:step:maxDist;
    distsAlongRay = flipVecToColumn(distsAlongRay);
    nodesAlongRay = distsAlongRay*rayDirn;
    nodesAlongRay = bsxfun(@plus,nodesAlongRay,rayOrigin);
end