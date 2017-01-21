function [dirn,segmentLength] = calcRayDirn(rayStart,rayEnd)
    dirn = rayEnd-rayStart;
    dirn = dirn/norm(dirn);
    segmentLength = norm(rayEnd-rayStart);
end