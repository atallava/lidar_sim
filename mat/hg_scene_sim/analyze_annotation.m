% i noticed that scene objects were buried in the ground
% lead to correcting ground transform

%% relpath helpers
genRelPathPrimitive = @(sectionId,className,elementId) ...
    sprintf('../data/sections/section_%02d/primitives/%s/%d.mat',sectionId,className,elementId);

genRelPathSceneAnnotation = @(sectionId) ...
    sprintf('../data/sections/section_%02d/scene_annotation',sectionId);

genRelPathClassPrimitivesDir = @(className) ...
    sprintf('../data/3d_models/primitives/%s',className);

% scene object meshes
genRelPathSceneTriModels = @(sectionId,triModelsId) ...
    sprintf('../../cpp/data/sections/section_%02d/mesh_model_sim/%d.txt',sectionId,triModelsId);

genRelPathSceneTriModelsMat = @(sectionId) ...
    sprintf('../data/sections/section_%02d/mesh_model_sim/scene_tri_models',sectionId);

genRelPathTriangleModelsMat = @(sectionId,blockId) ...
    sprintf('../data/sections/section_%02d/hg_sim/section_%02d_block_%02d_ground_triangles', ...
    sectionId,sectionId,blockId);

%% load
% annotations for section 4
trainSectionId = 3;
sectionId = 4;
relPathSceneAnnotation = genRelPathSceneAnnotation(sectionId);
load(relPathSceneAnnotation,'sceneAnnotation');

% class info
relPathPrimitiveClasses = '../data/primitive_classes';
load(relPathPrimitiveClasses,'primitiveClasses','primitiveClassIsPatch');

% triangle models
triBlockIds = 1:5;
triModelsGroundCell = cell(1,length(triBlockIds));
for i = 1:length(triBlockIds)
    blockId = triBlockIds(i);
    
    relPathTriModels = genRelPathTriangleModelsMat(sectionId,blockId);
    container = load(relPathTriModels,'triModels');
    triModelsGroundCell{i} = container.triModels;
end
triModelsGround = stitchTriModels(triModelsGroundCell);

%% pick an object
elementIdsPerClass = getPrimitiveElementIds(genRelPathClassPrimitivesDir,primitiveClasses);
nObjects = length(sceneAnnotation);

idx = 1;
objectAnnotation = sceneAnnotation{idx};
objectClass = objectAnnotation.objectClass;
className = primitiveClasses{objectClass};
sampledElementId = randsample(elementIdsPerClass{objectClass},1);
% load primitive
relPathPrimitive = genRelPathPrimitive(trainSectionId,className,sampledElementId);
primitiveContainer = load(relPathPrimitive,'ellipsoidModels','obb');
% transform triModels to the new pose
T = correctTransformForGround(objectAnnotation.T_object_to_world,objectAnnotation.objectObb_world,primitiveContainer.obb);
ellipsoidModels_world = applyTransfToEllipsoids(primitiveContainer.ellipsoidModels,T);

modelNbrRadius = 40;
triModelsGroundNbr = createTriModelsNbr(triModelsGround,objectAnnotation.objectObb_world.center,modelNbrRadius);

%% viz
hfig = figure;
% draw the object obb
drawObb(hfig,objectAnnotation.objectObb_world);
% draw axes
drawAxes3(hfig,objectAnnotation.T_object_to_world,10);
% plot the ellipsoids
drawEllipsoids(hfig,ellipsoidModels_world);
% plot some of the nearby ground
% drawTriModels(hfig,triModelsGroundNbr,'ground');



