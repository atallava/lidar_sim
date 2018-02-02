% preprocessing for mesh primitives

relPathSrcDirPly = '../../cpp/data/3d_models/src';
relPathSrcDirMat = '../data/3d_models/src';
srcTypes = {'shrubs','grass'};

clockLocal = tic();
for i = 1:length(srcTypes)
    srcType = srcTypes{i};
    fprintf('src type: %s\n',srcType);
    relPathDirPly = [relPathSrcDirPly '/' srcType];
    dirRes = dir(relPathDirPly);
    for j = 1:length(dirRes)
        fname = dirRes(j).name;
        if any(strcmp({'.','..'},fname))
            continue;
        else
            fprintf('fname: %s\n',fname);
            relPathPly = [relPathDirPly '/' fname];
            cmd = ['tail -2 ' relPathPly];
            model = loadPly(relPathPly);
            fnameMat = strrep(fname,'.ply','.mat');
            relPathMat = [relPathSrcDirMat '/' srcType '/' fnameMat];
            save(relPathMat,'model','relPathPly');
        end
    end
end
compTime = toc(clockLocal);
fprintf('comp time: %.2fs\n',compTime);
