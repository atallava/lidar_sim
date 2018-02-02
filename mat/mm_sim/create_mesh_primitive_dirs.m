% helper script

relPathPrimitiveClasses = '../data/primitive_classes';
load(relPathPrimitiveClasses,'primitiveClasses','primitiveClassIsPatch');

%%
relPathPre = '../data/3d_models/primitives';

for i = 1:length(primitiveClasses)
    relPathDir = [relPathPre '/' primitiveClasses{i}];
    mkdir(relPathDir);
end

%% for figs
relPathPre = '../figs/3d_models/primitives';

for i = 1:length(primitiveClasses)
    relPathDir = [relPathPre '/' primitiveClasses{i}];
    mkdir(relPathDir);
end

%% for cpp primitives
relPathPre = '../../cpp/data/3d_models/primitives';

for i = 1:length(primitiveClasses)
    relPathDir = [relPathPre '/' primitiveClasses{i}];
    mkdir(relPathDir);
end