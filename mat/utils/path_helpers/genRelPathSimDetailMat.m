function relPathSimDetail = genRelPathSimDetailMat(sectionId,simType,simVersion,queryType,tag)
%GENRELPATHSIMDETAILMAT
%
% relPathSimDetail = GENRELPATHSIMDETAILMAT(sectionId,simType,simVersion,queryType,tag)
%
% sectionId        -
% simType          -
% simVersion       -
% queryType        -
% tag              -
%
% relPathSimDetail -

if (nargin < 5)
    tag = -1;
end
relPathSimDetail = sprintf('../data/section_%02d/%s_sim/version_%s/%s_sim_detail', ...
    sectionId,simType,simVersion,queryType);

if (tag == -1)
    relPathSimDetail = [relPathSimDetail  '.txt'];
else
    relPathSimDetail = [relPathSimDetail '_' num2str(tag) '.txt'];
end
end