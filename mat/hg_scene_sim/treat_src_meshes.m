% warning: intended to run this only once
genRelPathModel = @(type,fname) ...
    sprintf('../data/3d_models/src/%s/%s',type,fname);

%%
srcTypes = {'trees','shrubs','grass'};
fnamesPerType = { ...
    {'Hyophorbe_lagenicaulis','Tree','Tree_Conifer_1','Tree_leaves','Tree_OBJ','trees'}, ...
    {'Tropical_Plant'}, ...
    {'Herbe','grass'}};

for srcTypeId = 1:length(srcTypes)
    fnamesThisType = fnamesPerType{srcTypeId};
    for fnameId = 1:length(fnamesThisType)
        srcType = srcTypes{srcTypeId};
        fname = fnamesThisType{fnameId};

        relPathModel = genRelPathModel(srcType,fname);
        load(relPathModel,'model');
        
        th = pi/2;
        T = eye(4);
        T(1:3,1:3) = rotx(pi/2);
        
        model = applyTransfToMeshModel(model,T);
        save(relPathModel,'model','-append');
    end
end

