function relPathDir = genRelPathMatchReportsDir(sectionId, scansVersion, dataSource, sourceVersion, paramSamplesVersion)
%GENRELPATHMATCHREPORTSDIR
%
% relPathDir = GENRELPATHMATCHREPORTSDIR(sectionId, scansVersion, dataSource, sourceVersion, paramSamplesVersion)
%
% sectionId           -
% scansVersion        -
% dataSource          -
% sourceVersion       -
% paramSamplesVersion -
%
% relPathDir          -

relPathDir = [genRelPathBagsDir(sectionId, scansVersion, dataSource, sourceVersion) ...
    '/match_reports/param_samples_version_' paramSamplesVersion];
end