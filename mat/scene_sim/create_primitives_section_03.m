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

%% load labeling
relPathPrimitiveClasses = '../data/primitive_classes';
load(relPathPrimitiveClasses,'primitiveClasses','primitiveClassIsPatch');

sectionId = 3;
relPathLabelingForSegmentIds = genRelPathLabelingForSegmentIds(sectionId);
load(relPathLabelingForSegmentIds,'labeling','segmentIds');

%% load ellipsoids
relPathDir = sprintf('../data/sections/section_%02d',sectionId);
pattern = sprintf('section_%02d_block_([0-9]+)_non_ground_ellipsoids.mat', ...
    sectionId);
[matchingFiles,blockIds] = getPatternMatchingFileIds(relPathDir,pattern);
ellipsoidModelsCell = cell(1,length(blockIds));
for i = 1:length(blockIds)
    relPathEllipsoids = genRelPathEllipsoids(sectionId,blockIds(i));
    can = load(relPathEllipsoids,'ellipsoidModels');
    ellipsoidModelsCell{i} = can.ellipsoidModels;
end
ellipsoidModels = stitchEllipsoidModels(ellipsoidModelsCell);

%%
nSegments = length(labeling);
nClasses = length(primitiveClasses);
classPrimitiveCount = zeros(1,nClasses);
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
        
        relPathPrimitive = genRelPathPrimitive(sectionId,segmentClassName, ...
            classPrimitiveCount(segmentClass));

        % pose of segment
        T_obb_to_world = getObbTransf(obb_world);
        % ellipsoids in obb
        obbEllipsoids_world = calcEllipsoidsInObb(ellipsoidModels,obb_world);
        % transform to identity
        T_world_to_obb = inv(T_obb_to_world);
        pts_obb = applyTransf(pts_world,T_world_to_obb);
        obb_obb = applyTransfToObb(obb_world,T_world_to_obb);
        obbEllipsoids_obb = applyTransfToEllipsoids(obbEllipsoids_world,T_world_to_obb);
        
        % write out
        can.sectionId = sectionId;
        can.segmentId = segmentId;
        can.classLabel = segmentClass;
        can.T_segment_to_world = T_obb_to_world;
        can.pts = pts_obb;
        can.obb = obb_obb;
        can.ellipsoidModels = obbEllipsoids_obb;
        
        relPathPrimitive = genRelPathPrimitive(sectionId,segmentClassName, ...
            classPrimitiveCount(segmentClass));
        save(relPathPrimitive,'-struct','can');
    else % is patch
        classPrimitiveCount(segmentClass) = classPrimitiveCount(segmentClass)+1;
        % make a directory for this patch
        relPathPrimitivePatch = ...
            genRelPathPrimitivePatch(sectionId,segmentClassName,classPrimitiveCount(segmentClass));
        mkdir(relPathPrimitivePatch);
        
        % get cell obbs
        [patchObbs,ptsInObbs] = calcPatchObbs(pts);
        nCellsInPatch = length(patchObbs);
        for j = 1:nCellsInPatch
            obb_world = patchObbs{j};
            pts_world = ptsInObbs{j};
            
            T_obb_to_world = getObbTransf(obb_world);
            % ellipsoids in obb
            obbEllipsoids_world = calcEllipsoidsInObb(ellipsoidModels,obb_world);
            % transform to identity
            T_world_to_obb = inv(T_obb_to_world);
            pts_obb = applyTransf(pts_world,T_world_to_obb);
            obb_obb = applyTransfToObb(obb_world,T_world_to_obb);
            obbEllipsoids_obb = applyTransfToEllipsoids(obbEllipsoids_world,T_world_to_obb);
            
            % write out
            can.sectionId = sectionId;
            can.segmentId = segmentId;
            can.classLabel = segmentClass;
            can.T_segment_to_world = T_obb_to_world;
            can.pts = pts_obb;
            can.obb = obb_obb;
            can.ellipsoidModels = obbEllipsoids_obb;
            
            relPathPrimitivePatchCell = genRelPathPrimitivePatchCell(sectionId,segmentClassName, ...
                classPrimitiveCount(segmentClass),j);
            save(relPathPrimitivePatchCell,'-struct','can');
        end
    end
    
    waitbar(i/nSegments);
end
compTime = toc(clockLocal);
fprintf('comp time: %.2fs\n',compTime);
