% run this script before creating primitives

%% relpath helpers
relPathPrimitiveClasses = '../data/primitive_classes';
load(relPathPrimitiveClasses,'primitiveClasses');

%% some info
sectionId = 3;
primitivesVersion = '080917';

%% make dirs
relPathPre = sprintf('../data/sections/section_%02d/primitives/version_%s',sectionId,primitivesVersion);
cmd = sprintf('mkdir %s',relPathPre);
system(cmd);

for i = 1:length(primitiveClasses)
    relPathDir = sprintf('%s/%s',relPathPre,primitiveClasses{i});
    cmd = sprintf('mkdir %s',relPathDir);
    system(cmd);
end

%% make dirs for figs
relPathPre = sprintf('../figs/sections/section_%02d/primitives/version_%s',sectionId,primitivesVersion);
cmd = sprintf('mkdir %s',relPathPre);
system(cmd);

for i = 1:length(primitiveClasses)
    relPathDir = sprintf('%s/%s',relPathPre,primitiveClasses{i});
    cmd = sprintf('mkdir %s',relPathDir);
    system(cmd);
end

