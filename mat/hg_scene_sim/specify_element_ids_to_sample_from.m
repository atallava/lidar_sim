% this is to weed out some primitives which don't look good

%%
elementIdsToSampleFrom = {1:4, 1:2, 1:18, 1:10, ... % low shrub, patch, small shrub, patch
    [1:19 22:27], 1:12, [1:6 10], [1 3:5], ... % medium shrub, patch, large shrub, patch
    [1:23 27:32] [1:15 17 30:32] [1:6 9 15 19]}; % small tree, medium tree, large tree

sectionId = 3;
primitivesVersion = '250417';
relPathOutput = sprintf('../data/sections/section_%02d/primitives/version_%s/element_ids_to_sample_from', ...
    sectionId,primitivesVersion);

save(relPathOutput,'elementIdsToSampleFrom');

%%
elementIdsToSampleFrom = {1:2, 1:2, 1:18, 1:11, ... % low shrub, patch, small shrub, patch
    [1:19 21:27], 1:12, [1:3 5:7 9:10], [1:5], ... % medium shrub, patch, large shrub, patch
    [1:23 27:32] [1:17 20:32] [1:11 14:19]}; % small tree, medium tree, large tree

sectionId = 3;
primitivesVersion = '080917';
relPathOutput = sprintf('../data/sections/section_%02d/primitives/version_%s/element_ids_to_sample_from', ...
    sectionId,primitivesVersion);

save(relPathOutput,'elementIdsToSampleFrom');
