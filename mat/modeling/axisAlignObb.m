function obbAxAl = axisAlignObb(obb)
    obbAxAl = obb;
    obbAxAl.ax1 = [1 0];
    obbAxAl.ax2 = [0 1];
    obbAxAl.center = [0 0 0];
end