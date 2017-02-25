function dist = distBetweenPoseIds(poseLog,id1,id2)
    dist = 0;
    for i = id1:(id2-1)
	pose1 = poseLog(i,:);
	pose2 = poseLog(i+1,:);
	posn1 = posnFromImuPose(pose1);
	posn2 = posnFromImuPose(pose2);
	ds = norm(posn1-posn2);
	dist = dist+ds;
    end
end
