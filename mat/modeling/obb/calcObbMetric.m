function metric = calcObbMetric(obb1,obb2)
%CALCOBBMETRIC
%
% metric = CALCOBBMETRIC(obb1,obb2)
%
% obb1   -
% obb2   -
%
% metric -

extents1 = obb1.extents;
sides1 = extents1(:,2)-extents1(:,1);

extents2 = obb2.extents;
sides2 = extents2(:,2)-extents2(:,1);

metric = norm(extents1(:)-extents2(:));
% metric = norm(sides2-sides1);
end