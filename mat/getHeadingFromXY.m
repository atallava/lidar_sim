function headings = getHeadingFromXY(xy)
% xy: (nXY x 2) array.

nXy = size(xy,1);
nHeadings = nXy-1;
headings = zeros(nHeadings,1);
for i = 1:nHeadings
    dy = xy(i+1,2)-xy(i,2);
    dx = xy(i+1,1)-xy(i,1);
    headings(i) = atan2(dy,dx);
end

end