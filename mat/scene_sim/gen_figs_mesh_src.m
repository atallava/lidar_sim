%% helpers
genRelPathModel = @(type,fname) ...
    sprintf('../data/3d_models/src/%s/%s',type,fname);
genRelPathDirModels = @(type) ...
        sprintf('../data/3d_models/src/%s',type);
genRelPathModelFig = @(type,fname) ...
    sprintf('../figs/3d_models/src/%s/%s',type,fname);
genRelPathModelPng = @(type,fname) ...
    sprintf('../figs/3d_models/src/%s/%s.png',type,fname);

someUsefulPaths;
addpath([pathToM '/altmany-export_fig-5be2ca4']);

%%
srcTypes = {'trees','shrubs','grass'};

for i = 1:length(srcTypes)
    srcType = srcTypes{i};
    fprintf('src type: %s\n',srcType);
    relPathDirModels = genRelPathDirModels(srcType);
    dirRes = dir(relPathDirModels);
    for j = 1:length(dirRes)
        fname = dirRes(j).name;
        fprintf('fname: %s\n',fname);
        if any(strcmp({'.','..'},fname))
            continue;
        else
            relPathModel = genRelPathModel(srcType,fname);
            load(relPathModel,'model');
            hfig = vizMesh(model);
            set(hfig,'visible','off');
            
            fnameNoExt = strrep(fname,'.mat','');
            % fig
%             relPathModelFig = genRelPathModelFig(srcType,fnameNoExt);
%             savefig(hfig,relPathModelFig);
            
            % png
            relPathModelPng = genRelPathModelPng(srcType,fnameNoExt);
            export_fig(relPathModelPng,hfig);
            close(hfig);
        end
    end
end