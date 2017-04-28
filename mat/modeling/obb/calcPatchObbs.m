function [obbCell,obbPtsCell] = calcPatchObbs(pts)
    %CALCPATCHOBBS
    %
    % [obbCell,obbPtsCell] = CALCPATCHOBBS(pts)
    %
    % pts        -
    %
    % obbCell    -
    % obbPtsCell -

    % patch obbs in own frame
    globalObb = calcObb(pts);
    T_obb_to_world = getObbTransf(globalObb);
    T_world_to_obb = inv(T_obb_to_world);
    pts_obb = applyTransf(pts,T_world_to_obb);
    [obbCell_obb,obbPtsCell_obb] = calcPatchObbsAxAl(pts_obb);
    
    % transform to world
    [obbCell,obbPtsCell] = deal(cell(size(obbCell_obb)));
    for i = 1:length(obbCell)
        obbCell{i} = applyTransfToObb(obbCell_obb{i},T_obb_to_world);
        obbPtsCell{i} = applyTransf(obbPtsCell_obb{i},T_obb_to_world);
    end
end