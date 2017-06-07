% rel path helpers
genRelPathMeshModel = @(type,fname) ...
    sprintf('../data/3d_models/src/%s/%s',type,fname);

genRelPathDirMeshModels = @(type) ...
        sprintf('../data/3d_models/src/%s',type);
    
genRelPathMeshPrimitive = @(className,elementId) ...
    sprintf('../data/3d_models/primitives/%s/%d',className,elementId);

genRelPathMeshPrimitivePatchCell = @(className,elementId,cellId) ...
    sprintf('../data/3d_models/primitives/%s/%d/%d',...
    className,elementId,cellId);

%% load
relPathPrimitiveClasses = '../data/primitive_classes';
load(relPathPrimitiveClasses,'primitiveClasses','primitiveClassIsPatch');

relPathHtsPerClass = '../data/sections/section_03/primitives/hts_per_class';
load(relPathHtsPerClass,'htsPerClass');

srcTypePerClass = {'shrubs','grass','shrubs','grass','shrubs','grass','shrubs','grass', ...
    'trees','trees','trees'};

%%
clockLocal = tic();
nClasses = length(primitiveClasses);
classPrimitiveCount = zeros(1,nClasses);

for i = 1:nClasses
    className = primitiveClasses{i};
    srcType = srcTypePerClass{i};
    fprintf('class: %s, src: %s...\n',className,srcType);
    
    htsThisClass = htsPerClass{i};
    relPathDirMeshes = genRelPathDirMeshModels(srcType);
    dirRes = dir(relPathDirMeshes);
    for j = 1:length(dirRes)
        fname = dirRes(j).name;
        if any(strcmp({'.','..'},fname))
            continue;
        else
            fprintf('fname: %s\n',fname);
            classPrimitiveCount(i) = classPrimitiveCount(i)+1;

            % load mesh model
            fname = strrep(fname,'.mat','');
            relPathModel = genRelPathMeshModel(srcType,fname);
            load(relPathModel,'model');
            
            % transform to origin
            obb = calcObb(model.vertices);
            T_model_to_world = eye(4,4);
            T_model_to_world(1:3,4) = obb.center;
            T_world_to_model = inv(T_model_to_world);
            model = applyTransfToMeshModel(model,T_world_to_model);

            % scale
            htSample = randsample(htsThisClass,1);
            scale = htSample/range(obb.extents(3,:));
            Tscale = scale*eye(4,4);
            model = applyTransfToMeshModel(model,Tscale);
            obb.extents = scale*obb.extents;
            triModels = convertMeshToTriModels(model);
            
            relPathMeshPrimitive = genRelPathMeshPrimitive(className,classPrimitiveCount(i));
            if ~primitiveClassIsPatch(i)
                % save directly 
%                 save(relPathMeshPrimitive,'className','triModels','obb','T_model_to_world');
            else
                % mkdir
                mkdir(relPathMeshPrimitive);
                
                % get cell obs
                [patchObbs,ptsInObbs] = calcPatchObbs(triModels.ptsFit);
                nCellsInPatch = length(patchObbs);
                cellModels_world = cell(1,nCellsInPatch);
                cellModels_obb = cell(1,nCellsInPatch);
                for k = 1:nCellsInPatch
                    obb_world = patchObbs{k};
                    pts_world = ptsInObbs{k};
                    
                    T_obb_to_world = getObbTransf(obb_world);
                    % triModels in obb
                    obbTriModels_world = snapTriModelsToObb(triModels,obb_world);
                    cellModels_world{k} = convertTriModelsToMeshModel(obbTriModels_world);
                    
                    % transform to identity
                    T_world_to_obb = inv(T_obb_to_world);
                    pts_obb = applyTransf(pts_world,T_world_to_obb);
                    obb_obb = applyTransfToObb(obb_world,T_world_to_obb);
                    obbTriModels_obb = applyTransfToTriModels(obbTriModels_world,T_world_to_obb);
                    cellModels_obb{k} = convertTriModelsToMeshModel(obbTriModels_world);
                    
                    % save
                    relPathMeshPrimitivePatchCell = ...
                        genRelPathMeshPrimitivePatchCell(className,classPrimitiveCount(i),k);
                    can.className = className;
                    can.triModels = obbTriModels_obb;
                    can.obb = obb_obb;
                    can.T_model_to_world = T_obb_to_world;
                    save(relPathMeshPrimitivePatchCell,'-struct','can');
                end
            end
        end
    end
end
compTime = toc(clockLocal);
fprintf('comp time: %.2fs\n',compTime);

