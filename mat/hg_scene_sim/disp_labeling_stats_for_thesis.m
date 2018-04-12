% get total number of labeled instances, across scenes

%% load
relPathPrimitives = '../data/primitive_classes';
load(relPathPrimitives, 'primitiveClasses');

%% loop over sections
sectionIds = [1 3 4];

nPrimitives = length(primitiveClasses);
nLabeledPerClass = zeros(1, nPrimitives);
for i = 1:length(sectionIds)
    sectionId = sectionIds(i);
    relPathLabeling = sprintf('../data/sections/section_%02d/labeling/labeling_for_segment_ids', sectionId);
    load(relPathLabeling, 'labeling');
    for j = 1:nPrimitives
        nLabeledPerClass(j) = nLabeledPerClass(j) + sum(labeling == j);
    end
end

%% disp
for i = 1:nPrimitives
    fprintf('class %s: %d\n', primitiveClasses{i}, nLabeledPerClass(i));
end