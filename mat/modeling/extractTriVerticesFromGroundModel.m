function [triVert1,triVert2,triVert3] = extractTriVerticesFromGroundModel(groundModel)
%EXTRACTTRIVERTICESFROMGROUNDMODEL 
% 
% [triVert1,triVert2,triVert3] = EXTRACTTRIVERTICESFROMGROUNDMODEL(groundModel)
% 
% groundModel - struct. ('tri','ptsFit',...)
% 
% triVert1    - [nTri,3] array.
% triVert2    - [nTri,3] array.
% triVert3    - [nTri,3] array.

tri = groundModel.tri;
pts = groundModel.ptsFit;

triVert1 = pts(tri(:,1),:);
triVert2 = pts(tri(:,2),:);
triVert3 = pts(tri(:,3),:);
end