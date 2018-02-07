function elementIdsPerClass = getPrimitiveElementIds()
%GETPRIMITIVEELEMENTIDS Element ids per class.
%
% elementIdsPerClass = GETPRIMITIVEELEMENTIDS()
%
% elementIdsPerClass - [1,nClasses] cell array.

% all elements
% elementIdsPerClass = { 1:5, 1, 1:5, 1, 1:5, 1, 1:5, 1, ...
%     1:16, 1:16, 1:16};

% elements of 'small' size
largeSizeShrubIds = 5; % over 500kB. shrubs get used many times over
smallSizeShrubIds = setdiff(1:5,largeSizeShrubIds);
largeSizeTreeIds = [10 1 2 6 16]; % files over 2MB
smallSizeTreeIds = setdiff(1:16,largeSizeTreeIds);
elementIdsPerClass = {smallSizeShrubIds, 1, smallSizeShrubIds, 1, smallSizeShrubIds, 1, smallSizeShrubIds, 1, ...
    smallSizeTreeIds, smallSizeTreeIds, smallSizeTreeIds};
end