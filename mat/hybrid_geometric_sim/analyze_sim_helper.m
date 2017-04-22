 home;
 [originId,dirnId] = findMatchInSimDetail(ptsRealCell,cursor_info.Position)
 hitFlagCell{originId}(dirnId)
 rayOrigins(originId,:)
 ptsRealCell{originId}(dirnId,:)
 ptsSimCell{originId}(dirnId,:)
 printVecAsStlInput(rayOrigins(originId,:))
 printVecAsStlInput(calcRayDirn(rayOrigins(originId,:),ptsRealCell{originId}(dirnId,:)))
 