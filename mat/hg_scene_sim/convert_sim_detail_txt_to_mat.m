% helper script

%% load txt
sectionId = 4;
simType = 'hg';
simVersion = '080917';
queryType = 'slice';
tag = -1;
relPathSimDetailTxt = genRelPathSimDetailTxt(sectionId,simType,simVersion,queryType,tag);

clockLocal = tic();
simDetail = loadSimDetail(relPathSimDetailTxt);
elapsedTime = toc(clockLocal);
fprintf('elapsed time: %.2fs\n',elapsedTime);

%% 
relPathSimDetailMat = genRelPathSimDetailMat(sectionId,simType,simVersion,queryType,tag);
save(relPathSimDetailMat,'simDetail');