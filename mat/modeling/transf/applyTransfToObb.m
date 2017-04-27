function obb2 = applyTransfToObb(obb1,T_1_to_2)
    center1 = obb1.center;
    center2 = applyTransf(center1,T_1_to_2);
    obb2 = obb1;
    obb2.center = center2;
    
    R = T_1_to_2(1:3,1:3);
    
    ax1_1 = [obb1.ax1; 1];
    ax1_2 = R*ax1_1;
    obb2.ax1 = ax1_2(1:2);
    
    ax2_1 = [obb1.ax2; 1];
    ax2_2 = R*ax2_1;
    obb2.ax2 = ax2_2(1:2);
end