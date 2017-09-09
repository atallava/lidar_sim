function [objectPts,objectEllipsoids] = constructSceneObject(objectAnnotation,primitives)
%CONSTRUCTSCENEOBJECT
%
% [objectPts,objectEllipsoids] = CONSTRUCTSCENEOBJECT(objectAnnotation,primitives)
%
% objectAnnotation -
% primitives       -
%
% objectPts        -
% objectEllipsoids -

% extract primitive obbs
nPrimitives = length(primitives);
primitiveObbs = cell(1,nPrimitives);
for i = 1:nPrimitives
    primitiveObbs{i} = primitives{i}.obb;
end

% pick element by matching obb
elementId = pickNearestObb(objectAnnotation.objectObb_world,primitiveObbs);

% pick element random sample
% elementId = randsample(nPrimitives,1);

selectedPrimitive = primitives{elementId};

% transform primitive to scene
TCorrected = correctTransformForGround(objectAnnotation.T_object_to_world,objectAnnotation.objectObb_world, ...
    selectedPrimitive.obb);
objectPts = applyTransf(selectedPrimitive.pts,TCorrected);
objectEllipsoids = applyTransfToEllipsoids(selectedPrimitive.ellipsoidModels,TCorrected);
end