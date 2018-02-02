function [ellipsoidModels_world,pts_world] = createObjectFromPrimitive(objectAnnotation,primitive)
%CREATEOBJECTFROMPRIMITIVE
%
% [ellipsoidModels_world,pts_world] = CREATEOBJECTFROMPRIMITIVE(objectAnnotation,primitive)
%
% objectAnnotation      -
% primitive             -
%
% ellipsoidModels_world -
% pts_world             -

pts = primitive.pts;
obb = primitive.obb;
ellipsoidModels = primitive.ellipsoidModels;

TCorrected = correctTransformForGround(objectAnnotation.T_object_to_world,objectAnnotation.objectObb_world,obb);
pts_world = applyTransf(pts,TCorrected);
ellipsoidModels_world = applyTransfToEllipsoids(ellipsoidModels,TCorrected);
end