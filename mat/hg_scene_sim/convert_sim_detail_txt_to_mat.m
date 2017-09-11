% helper script

%% load txt
sectionId = 4;
simType = 'hg';
simVersion = '280817';
queryType = 'slice';
tag = -1;
relPathSimDetailTxt = genRelPathSimDetailTxt(sectionId,simType,simVersion,queryType,tag);

simDetail = loadSimDetail(relPathSimDetailTxt);

%% 
relPathSimDetailMat = genRelPathSimDetailMat(sectionId,tag);
save(relPathSimDetailMat,'simDetail');