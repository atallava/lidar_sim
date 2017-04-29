relPathPrimitiveClasses = '../data/primitive_classes';
load(relPathPrimitiveClasses,'primitiveClasses');

%%
relPathPre = '../data/sections/section_03/primitives';
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


