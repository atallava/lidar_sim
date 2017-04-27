function maxExtent = calcObbMaxExtent(obb)
    %CALCOBBMAXEXTENT
    %
    % maxExtent = CALCOBBMAXEXTENT(obb)
    %
    % obb       - struct.
    %
    % maxExtent - scalar.
    
    maxExtent = max(abs(obb.extents(:)));
end