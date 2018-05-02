function relPathReports = genRelPathMatchReports(sectionId, scansVersion, dataSource, sourceVersion, paramSamplesVersion, paramIdx)
%GENRELPATHMATCHREPORTS
%
% relPathReports = GENRELPATHMATCHREPORTS(sectionId, scansVersion, dataSource, sourceVersion, paramSamplesVersion, paramIdx)
%
% sectionId           -
% scansVersion        -
% dataSource          -
% sourceVersion       -
% paramSamplesVersion -
% paramIdx            -
%
% relPathReports      -

if nargin == 4
    relPathReports = [genRelPathBagsDir(sectionId, scansVersion, dataSource, sourceVersion) '/reports.txt'];
else
    relPathReports = [genRelPathMatchReportsDir(sectionId, scansVersion, dataSource, sourceVersion, paramSamplesVersion) ...
        '/reports_' num2str(paramIdx) '.txt'];
end
end