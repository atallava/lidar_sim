function pts = getSectionPtsAtTime(section,t)
    pts = section.pts(section.ptTimestamps == t,:);
end