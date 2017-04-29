function createAndSavePrimitive(pts,ellipsoidModels,segmentId,saveData)
    % primitive in world frame
    obb_world = calcObb(pts);
    obbEllipsoids_world = calcEllipsoidsInObb(ellipsoidModels,obb_world);
    if isfield(saveData,'relPathPrimitiveWorldFrame')
        clear('can');
        can.pts = pts;
        can.obb = obb_world;
        can.ellipsoidModels = obbEllipsoids_world;
        can.segmentId = segmentId;
        save(saveData.relPathPrimitiveWorldFrame,'-struct','can');
    end
    
    % obb frame
    T_obb_to_world = getObbTransf(obb_world);
    T_world_to_obb = inv(T_obb_to_world);
    pts_obb = applyTransf(pts,T_world_to_obb);
    obb = applyTransfToObb(obb_world,T_world_to_obb);
    obbEllipsoids = applyTransfToEllipsoids(obbEllipsoids_world,T_world_to_obb);
    if isfield(saveData,'relPathPrimitive')
        clear('can');
        can.pts = pts_obb;
        can.obb = obb;
        can.ellipsoidModels = obbEllipsoids;
        can.segmentId = segmentId;
        save(saveData.relPathPrimitive,'-struct','can');
    end
    
    % figures
    plotStructVars = {'ellipsoidData','plotStruct'};
    clear(plotStructVars{:});
    ellipsoidData.ellipsoidModels = obbEllipsoids;
    ellipsoidData.uniformAlpha = false;
    plotStruct.ellipsoidData = ellipsoidData;
    hfig = plotRangeData(plotStruct);
    set(hfig,'visible','off');
    drawObb(hfig,obb,pts_obb);
    if isfield(saveData,'relPathPrimitiveFig')
        savefig(hfig,saveData.relPathPrimitiveFig);
    end
    if isfield(saveData,'relPathPrimitivePng')
        export_fig(relPathFig,hfig);
    end
    
    close('hfig'); clear('hfig');
end