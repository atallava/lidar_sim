function T = getObbTransf(obb)
    theta = atan2(obb.ax1(2),obb.ax1(1));
    R = rotz(theta);
    T = eye(4);
    T(1:3,1:3) = R;
    T(1:3,4) = obb.center;
end