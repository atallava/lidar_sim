function saveClassPrimitive(segmentId,pts,ellipsoidModels,relPathPrimitive)
%SAVECLASSPRIMITIVE
%
% SAVECLASSPRIMITIVE(segmentId,pts,ellipsoidModels,relPathPrimitive)
%
% segmentId        -
% pts              -
% ellipsoidModels  -
% relPathPrimitive -

% calc obb
obb_world = calcObb(pts);
pts_world = pts; % just a rewording

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
save(relPathPrimitive,'-struct','can');
end