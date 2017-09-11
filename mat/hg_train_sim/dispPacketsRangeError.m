function dispPacketsRangeError(simDetail,packetIdxs)
simDetailNew = simDetail;
simDetailNew.rayOrigins = simDetail.rayOrigins(packetIdxs,:);
simDetailNew.rayPitchesCell = simDetail.rayPitchesCell(packetIdxs);
simDetailNew.rayYawsCell = simDetail.rayYawsCell(packetIdxs);
simDetailNew.realPtsAllCell = simDetail.realPtsAllCell(packetIdxs);
simDetailNew.realHitFlagCell = simDetail.realHitFlagCell(packetIdxs);
simDetailNew.simPtsAllCell = simDetail.simPtsAllCell(packetIdxs);
simDetailNew.simHitFlagCell = simDetail.simHitFlagCell(packetIdxs);

dispRangeError(simDetailNew);
end