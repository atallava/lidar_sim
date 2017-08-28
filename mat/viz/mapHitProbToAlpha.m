function transparency = mapHitProbToAlpha(hitProb)
    alphaLow = 0;
    alphaHigh = 0.5;
    transparency = alphaLow + hitProb.*alphaHigh;
end