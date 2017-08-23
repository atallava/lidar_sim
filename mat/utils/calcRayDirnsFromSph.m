function rayDirns = calcRayDirnsFromSph(rayPitches,rayYaws)
rayPitches = flipVecToColumn(rayPitches);
rayYaws = flipVecToColumn(rayYaws);
rayDirns = [cos(rayPitches).*cos(rayYaws) cos(rayPitches).*sin(rayYaws) sin(rayPitches)]; 
end