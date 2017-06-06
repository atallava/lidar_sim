% rel path helpers
genRelPathPrimitive = @(sectionId,className,elementId) ...
    sprintf('../data/sections/section_%02d/primitives/%s/%d.mat',sectionId,className,elementId);

genRelPathPrimitivePatch = @(sectionId,className,elementId) ...
    sprintf('../data/sections/section_%02d/primitives/%s/%d',sectionId,className,elementId);

genRelPathPrimitivePatchCell = @(sectionId,className,elementId,cellId) ...
    sprintf('../data/sections/section_%02d/primitives/%s/%d/%d.mat',...
    sectionId,className,elementId,cellId);

genRelPathClassPrimitivesDir = @(sectionId,className) ...
    sprintf('../data/sections/section_%02d/primitives/%s',sectionId,className);

%% load labeling
relPathPrimitiveClasses = '../data/primitive_classes';
load(relPathPrimitiveClasses,'primitiveClasses','primitiveClassIsPatch');

%% calc hts
sectionId = 3;
genRelPathClassPrimitivesDir2 = @(className) ...
    genRelPathClassPrimitivesDir(sectionId,className);
elementIdsPerClass = getPrimitiveElementIds(genRelPathClassPrimitivesDir2,primitiveClasses);

nClasses = length(primitiveClasses);
htsPerClass = cell(1,nClasses);
waitbar(0,'progress');
for i = 1:nClasses
    className = primitiveClasses{i};
    elementIds = elementIdsPerClass{i};
    nElements = length(elementIds);
    hts = [];
    for j = 1:nElements
        elementId = elementIds(j);
        if ~primitiveClassIsPatch(i)
            % load primitive
            relPathPrimitive = genRelPathPrimitive(sectionId,className,elementId);
            load(relPathPrimitive,'pts');
            obb = calcObb(pts);
            hts = [hts range(obb.extents(3,:))];
        else
            relPathPrimitivePatch = genRelPathPrimitivePatch(sectionId,className,elementId);
            pattern = '([0-9]+)';
            [~,primitiveCellIds] = getPatternMatchingFileIds(relPathPrimitivePatch,pattern);
            nCells = length(primitiveCellIds);
            for k = 1:nCells
                cellId = primitiveCellIds(k);
                relPathPrimitivePatchCell = genRelPathPrimitivePatchCell(sectionId,className,elementId,cellId);
                load(relPathPrimitivePatchCell,'pts');
                obb = calcObb(pts);
                hts = [hts range(obb.extents(3,:))];
            end
        end
        htsPerClass{i} = hts;
    end
    waitbar(i/nClasses);
end

%% viz
for i = 1:nClasses
    hts = htsPerClass{i};
    fprintf('class %s:\n',primitiveClasses{i});
    fprintf('hts mean: %.2f, std: %.2f\n',mean(hts),std(hts));
end

%% save
relPathOut = 'hts_per_class';
save(relPathOut,'htsPerClass');

