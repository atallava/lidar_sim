% as an accompaniment to scene models
sectionId = 42;

someUsefulPaths;
addpath([pathToM '/distinguishable_colors']);

%% load
% class info
relPathPrimitiveClasses = '../data/primitive_classes';
load(relPathPrimitiveClasses,'primitiveClasses');

% annotations
relPathAnnotation = sprintf('../data/sections/section_%02d/scene_annotation', sectionId);
load(relPathAnnotation, 'sceneAnnotation');

nClasses = 11;
colorPerClass = distinguishable_colors(nClasses);

%% viz
vizSceneAnnotation(sceneAnnotation, primitiveClasses, colorPerClass);
