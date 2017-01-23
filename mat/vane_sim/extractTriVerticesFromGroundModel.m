function [triVert1,triVert2,triVert3] = extractTriVerticesFromGroundModel(groundModel)
tri = groundModel.tri;
pts = groundModel.ptsFit;

triVert1 = pts(tri(:,1),:);
triVert2 = pts(tri(:,2),:);
triVert3 = pts(tri(:,3),:);
end