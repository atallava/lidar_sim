function relPathHgModelsDirMat = genRelPathHgModelsDirMat(sectionId,simVersion)
%GENRELPATHHGMODELSDIRMAT
%
% relPathHgModelsDirMat = GENRELPATHHGMODELSDIRMAT(sectionId,simVersion)
%
% sectionId             -
% simVersion            -
%
% relPathHgModelsDirMat -

relPathHgModelsDirMat = sprintf('../data/sections/section_%02d/hg_sim/version_%s', ...
    sectionId,simVersion);
end