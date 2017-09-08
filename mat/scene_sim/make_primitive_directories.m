relPathPrimitiveClasses = '../data/primitive_classes';
load(relPathPrimitiveClasses,'primitiveClasses');

%% 
sectionId = 3;
primitivesVersion = '080917';

%%
relPathPre = sprintf('../data/sections/section_%02d/primitives/version_%s',sectionId,primitivesVersion);
cmd = sprintf('mkdir %s',relPathPre);
system(cmd);

for i = 1:length(primitiveClasses)
    relPathDir = sprintf('%s/%s',relPathPre,primitiveClasses{i});
    cmd = sprintf('mkdir %s',relPathDir);
    system(cmd);
end

%%
relPathPre = '../figs/sections/section_03/primitives';
for i = 1:length(primitiveClasses)
    relPathDir = sprintf('%s/%s',relPathPre,primitiveClasses{i});
    cmd = sprintf('mkdir %s',relPathDir);
    system(cmd);
end


