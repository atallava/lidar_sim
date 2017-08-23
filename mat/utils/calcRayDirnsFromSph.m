function rayDirns = calcRayDirnsFromSph(rayPitches,rayYaws)
%CALCRAYDIRNSFROMSPH
%
% rayDirns = CALCRAYDIRNSFROMSPH(rayPitches,rayYaws)
%
% rayPitches -
% rayYaws    -
%
% rayDirns   -

rayPitches = flipVecToColumn(rayPitches);
rayYaws = flipVecToColumn(rayYaws);
rayDirns = [cos(rayPitches).*cos(rayYaws) cos(rayPitches).*sin(rayYaws) sin(rayPitches)];
end