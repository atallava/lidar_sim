function [sortedIntersectingIds,sortedDistAlongRayIntersections] = sortIntersectionFlag(intersectionFlag,distAlongRay)
    %SORTINTERSECTIONFLAG
    %
    % [sortedIntersectingIds,sortedDistAlongRayIntersections] = SORTINTERSECTIONFLAG(intersectionFlag,distAlongRay)
    %
    % intersectionFlag                - nEllipsoids length array. logical
    % array, intersection or not.
    % distAlongRay                    - nEllipsoids length array. dist
    % along ray for minimum maha dist with ellipsoids.
    %
    % sortedIntersectingIds           - nIntersectingEllipsoids length
    % array. ids of intersecting ellipsoids, sorted in increasing dist.
    % sortedDistAlongRayIntersections - nIntersectingEllipsoids length
    % array. sorted distances along ray.

    intersectingIds = find(intersectionFlag);
    distAlongRayIntersections = distAlongRay(intersectingIds);
    [sortedDistAlongRayIntersections,sortedIds] = sort(distAlongRayIntersections);
    sortedIntersectingIds = intersectingIds(sortedIds);
end