exp1Cell = {'genRelPathPosesLog(','genRelPathImuPosnNodes(','genRelPathBlockNodeIdsGround(', ...
       'genRelPathBlockNodeIdsNonGround(','genRelPathNonGroundBlockPts(','genRelPathGroundBlockPts(','genRelPathHgModelsDir('};

exp2Cell = {'genPathPosesLog(','genPathImuPosnNodes(','genPathBlockNodeIdsGround(', ...
       'genPathBlockNodeIdsNonGround(','genPathNonGroundBlockPts(','genPathGroundBlockPts(','genPathHgModelsDir('}; 

nChanges = length(exp1Cell);

dirpathCell = {'/usr0/home/atallav1/lidar_sim/cpp/src', ...
    '/usr0/home/atallav1/lidar_sim/cpp/hybrid_geometric_sim', ...
    '/usr0/home/atallav1/lidar_sim/cpp/scene_sim'};

%%
for i = 2:nChanges
    exp1 = exp1Cell{i};
    exp2 = exp2Cell{i};
    for j = 1:length(dirpathCell)
        dirpath = dirpathCell{j};
        cd(dirpath);
        cmd = sprintf('grep -rl ''%s'' | xargs sed -i.bak ''s/%s/%s/g'' ', ...
            exp1,exp1,exp2);
        [status,cmdout] = system(cmd);
    end
end


