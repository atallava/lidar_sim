function relPathObject = genRelPathSceneMeshObjectCpp(sectionId, simVersion, objectId)
%GENRELPATHSCENEMESHOBJECTCPP
%
% relPathObject = GENRELPATHSCENEMESHOBJECTCPP(sectionId, simVersion, objectId)
%
% sectionId     -
% simVersion    -
% objectId      -
%
% relPathObject -

relPathDir = mm_utils.genRelPathSceneMeshObjectCppDir(sectionId, simVersion);
objectFname = sprintf('%d.txt',objectId);
relPathObject = [relPathDir '/' objectFname];
end