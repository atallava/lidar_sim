% useful stats like num ell, num pts, heights

%% rel path helpers
genRelPathPrimitiveStats = @(sectionId,primitivesVersion) ...
    sprintf('../data/sections/section_%02d/primitives/version_%s/primitive_stats',sectionId,primitivesVersion);

%% load labeling
relPathPrimitiveClasses = '../data/primitive_classes';
load(relPathPrimitiveClasses,'primitiveClasses','primitiveClassIsPatch');

%% calc stats
sectionId = 3;
primitivesVersion = '080917';
elementIdsPerClass = getPrimitiveElementIds(sectionId,primitivesVersion,primitiveClasses);

nClasses = length(primitiveClasses);
[nElementsPerClass,nCellsPerClass] = deal(zeros(1,nClasses));
[htsPerClass,nElementPtsPerClass,nElementEllipsoidsPerClass] = ...
    deal(cell(1,nClasses));
waitbar(0,'progress');
for i = 1:nClasses
    className = primitiveClasses{i};
    elementIds = elementIdsPerClass{i};
    nElements = length(elementIds);
    nElementsPerClass(i) = nElements;
    [hts,nPts,nEllipsoids] = deal([]);
    for j = 1:nElements
        elementId = elementIds(j);
        if ~primitiveClassIsPatch(i)
            % load primitive
            relPathPrimitive = genRelPathPrimitive(sectionId,primitivesVersion,className,elementId);
            load(relPathPrimitive,'pts','obb','ellipsoidModels');
            hts = [hts range(obb.extents(3,:))];
            nPts = [nPts size(pts,1)];
            nEllipsoids = [nEllipsoids length(ellipsoidModels)];
        else
            relPathPrimitivePatch = genRelPathPatchPrimitive(sectionId,primitivesVersion,className,elementId);
            pattern = '([0-9]+)';
            [~,primitiveCellIds] = getPatternMatchingFileIds(relPathPrimitivePatch,pattern);
            nCells = length(primitiveCellIds);
            nCellsPerClass(i) = nCellsPerClass(i) + nCells;
            for k = 1:nCells
                cellId = primitiveCellIds(k);
                relPathPrimitivePatchCell = genRelPathPatchPrimitiveCell(sectionId,primitivesVersion,className,elementId,cellId);
                load(relPathPrimitive,'pts','obb','ellipsoidModels');
                hts = [hts range(obb.extents(3,:))];
                nPts = [nPts size(pts,1)];
                nEllipsoids = [nEllipsoids length(ellipsoidModels)];
            end
        end
        htsPerClass{i} = hts;
        nElementPtsPerClass{i} = nPts;
        nElementEllipsoidsPerClass{i} = nEllipsoids;
    end
    waitbar(i/nClasses);
end

%% viz
for i = 1:nClasses
    fprintf('class %s:\n',primitiveClasses{i});
    fprintf('n elements: %d, n cells: %d\n',nElementsPerClass(i),nCellsPerClass(i));
    nElementPts = nElementPtsPerClass{i};
    fprintf('n element pts. mean: %.2f, std: %.2f\n',mean(nElementPts),std(nElementPts));
    nElementEllipsoids = nElementEllipsoidsPerClass{i};
    fprintf('n element ellipsoids. mean: %.2f, std: %.2f\n', ...
        mean(nElementEllipsoids),std(nElementEllipsoids));
    hts = htsPerClass{i};
    fprintf('hts mean: %.2f, std: %.2f\n',mean(hts),std(hts));
end

%% save
relPathOut = genRelPathPrimitiveStats(sectionId,primitivesVersion);
save(relPathOut,'nElementsPerClass','nCellsPerClass', ...
    'nElementPtsPerClass','nElementEllipsoidsPerClass','htsPerClass');

