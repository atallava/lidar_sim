% rel path helpers
genRelPathMeshPrimitive = @(className,elementId) ...
    sprintf('../data/3d_models/primitives/%s/%d',className,elementId);

%% load
relPathPrimitiveClasses = '../data/primitive_classes';
load(relPathPrimitiveClasses,'primitiveClasses','primitiveClassIsPatch');

%% pick primitive
className = 'small_tree';
elementId = 10;
relPathPrimitive = genRelPathMeshPrimitive(className,elementId);

load(relPathPrimitive,'triModels');

fracn = 0.1;
triModelsReduced = reduceTriModels(triModels,fracn);

%%
hfig1 = figure();
drawTriModels(hfig1,triModels);
title(sprintf('class name: %s, element id: %d',className,elementId));
view(3);

hfig2 = figure();
drawTriModels(hfig2,triModelsReduced);
title(sprintf('class name: %s, element id: %d, reduced',className,elementId));
view(3);
