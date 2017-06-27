%% rel path helpers
% sources
genRelPathLabelingForSegmentIds = @(sectionId) ...
    sprintf('../data/sections/section_%02d/labeling/labeling_for_segment_ids',sectionId);

genRelPathPtsMat = @(sectionId,segmentId) ...
    sprintf('../data/sections/section_%02d/non_ground_segmentation/%d.mat', ...
    sectionId,segmentId);

genRelPathEllipsoids = @(sectionId,blockId) ...
    sprintf('../data/sections/section_%02d/section_%02d_block_%02d_non_ground_ellipsoids.mat', ...
    sectionId,sectionId,blockId);

% primitives
genRelPathPrimitive = @(sectionId,className,elementId) ...
    sprintf('../data/sections/section_%02d/primitives/%s/%d.mat',sectionId,className,elementId);

genRelPathPrimitivePatch = @(sectionId,className,elementId) ...
    sprintf('../data/sections/section_%02d/primitives/%s/%d',sectionId,className,elementId);

genRelPathPrimitivePatchCell = @(sectionId,className,elementId,cellId) ...
    sprintf('../data/sections/section_%02d/primitives/%s/%d/%d.mat',...
    sectionId,className,elementId,cellId);

% annotation
genRelPathSceneAnnotation = @(sectionId) ...
    sprintf('../data/sections/section_%02d/scene_annotation',sectionId);

%% load labeling
relPathPrimitiveClasses = '../data/primitive_classes';
load(relPathPrimitiveClasses,'primitiveClasses','primitiveClassIsPatch');

sectionId = 4;
relPathLabelingForSegmentIds = genRelPathLabelingForSegmentIds(sectionId);
load(relPathLabelingForSegmentIds,'labeling','segmentIds');

%% create annotation from labeling
nSegments = length(labeling);
nClasses = length(primitiveClasses);
classPrimitiveCount = zeros(1,nClasses);
nLabeledSegments = sum(~(~labeling));
sceneAnnotation = cell(1,nLabeledSegments);
objectCount = 0;
hWaitbar = waitbar(0,'progress');

clockLocal = tic();
for i = 1:nSegments
    segmentClass = labeling(i);
    if ~segmentClass
        continue;
    end
    segmentClassName = primitiveClasses{segmentClass};
    segmentId = segmentIds(i);
    
    fprintf('segment: %d, class: %s...\n',segmentId,segmentClassName);
    
    % load segment pts
    relPathPts = genRelPathPtsMat(sectionId,segmentId);
    load(relPathPts,'pts');
    
    % if not patch
    if ~primitiveClassIsPatch(segmentClass)
        classPrimitiveCount(segmentClass) = classPrimitiveCount(segmentClass)+1;

        % calc obb
        obb_world = calcObb(pts);
        pts_world = pts; % just a rewording
        % pose of segment
        T_obb_to_world = getObbTransf(obb_world);
        
        clear('objectAnnotation');
        objectAnnotation.objectClass = segmentClass;
        objectAnnotation.objectObb_world = obb_world;
        objectAnnotation.T_object_to_world = T_obb_to_world;
        objectCount = objectCount+1;
        sceneAnnotation{objectCount} = objectAnnotation;
    else % is patch
        classPrimitiveCount(segmentClass) = classPrimitiveCount(segmentClass)+1;
            
        % get cell obbs
        [cellObbs_world,cellPts_world] = calcPatchObbs(pts);
        nCellsInPatch = length(cellObbs_world);
        T_cells_to_world = cell(1,nCellsInPatch);
        for j = 1:nCellsInPatch
            T_obb_to_world = getObbTransf(cellObbs_world{j});
            T_cells_to_world{j} = T_obb_to_world;
        end
        
        clear('objectAnnotation');
        objectAnnotation.objectClass = segmentClass;
        objectAnnotation.cellObbs_world = cellObbs_world;
        objectAnnotation.T_cells_to_world = T_cells_to_world;
        objectCount = objectCount+1;
        sceneAnnotation{objectCount} = objectAnnotation;
    end
    
    waitbar(i/nSegments);
    waitbarTitle = sprintf('progress: %d/%d',i,nSegments);
    setWaitbarTitle(hWaitbar,waitbarTitle);
end
compTime = toc(clockLocal);
fprintf('comp time: %.2fs\n',compTime);

%% save
relPathSceneAnnotation = genRelPathSceneAnnotation(sectionId);
save(relPathSceneAnnotation,'sceneAnnotation');