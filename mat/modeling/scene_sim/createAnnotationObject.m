function annotation = createAnnotationObject(segmentClass,pts)
%CREATEANNOTATIONOBJECT
%
% annotation = CREATEANNOTATIONOBJECT(segmentClass,pts)
%
% segmentClass - scalar.
% pts          - [nPts,3] array. Assumed to be in world frame.
%
% annotation   - struct.

% calc obb
obb_world = calcObb(pts);
pts_world = pts; % just a rewording
% pose of segment
T_obb_to_world = getObbTransf(obb_world);

annotation.objectClass = segmentClass;
annotation.objectObb_world = obb_world;
annotation.T_object_to_world = T_obb_to_world;
end