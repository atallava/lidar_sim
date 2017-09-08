function [ellipsoidModels_world,pts_world] = createObjectFromPrimitive(objectAnnotation,primitive)
pts = primitive.pts;
obb = primitive.obb;
ellipsoidModels = primitive.ellipsoidModels;

TCorrected = correctTransformForGround(objectAnnotation.T_object_to_world,objectAnnotation.objectObb_world,obb);
pts_world = applyTransf(pts,TCorrected);
ellipsoidModels_world = applyTransfToEllipsoids(ellipsoidModels,TCorrected);
end