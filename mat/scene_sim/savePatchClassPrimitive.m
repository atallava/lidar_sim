function savePatchClassPrimitive(segmentId,pts,ellipsoidModels,relPathPatchPrimitive)

% calc cell obbs
[patchObbs,ptsInObbs] = calcPatchObbs(pts);
nCellsInPatch = length(patchObbs);

% loop over cells
for i = 1:nCellsInPatch
    obb_world = patchObbs{i};
    pts_world = ptsInObbs{i};
    
    % pose of segment
    T_obb_to_world = getObbTransf(obb_world);
    % ellipsoids in obb
    obbEllipsoids_world = calcEllipsoidsInObb(ellipsoidModels,obb_world);
    
    % transform to identity
    T_world_to_obb = inv(T_obb_to_world);
    pts_obb = applyTransf(pts_world,T_world_to_obb);
    obb_obb = applyTransfToObb(obb_world,T_world_to_obb);
    obbEllipsoids_obb = applyTransfToEllipsoids(obbEllipsoids_world,T_world_to_obb);
    
    % wrap in struct
    can.segmentId = segmentId;
    can.T_segment_to_world = T_obb_to_world;
    can.pts = pts_obb;
    can.obb = obb_obb;
    can.ellipsoidModels = obbEllipsoids_obb;
    
    % save
    relPathPrimitivePatchCell = [relPathPatchPrimitive '/' num2str(i)];
    save(relPathPrimitivePatchCell,'-struct','can');
end
end